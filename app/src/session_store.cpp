#include "session_store.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace {
constexpr int maximumDatagramBytes = static_cast<int>(BUFFER_LEN);
constexpr std::size_t commitBatchSize = 128;
constexpr int currentSessionSchemaVersion = 2;

#pragma pack(push, 1)
struct LegacyPacketHeaderV1 {
    std::uint16_t length;
    PacketType type;
    PacketID id;
    std::uint8_t seq;
};
#pragma pack(pop)
static_assert(sizeof(LegacyPacketHeaderV1) == 5);

void bindTransientText(sqlite3_stmt* statement, int index,
                       const std::string& value) {
    sqlite3_bind_text(statement, index, value.c_str(),
                      static_cast<int>(value.size()), SQLITE_TRANSIENT);
}

std::string pathToUtf8(const std::filesystem::path& path) {
    const std::u8string encoded = path.u8string();
    std::string result(encoded.size(), '\0');
    if(!encoded.empty()) {
        std::memcpy(result.data(), encoded.data(), encoded.size());
    }
    return result;
}
} // namespace

SessionStore::~SessionStore() {
    stopRecording();
    closeReplay();
    closeDatabase();
}

bool SessionStore::startRecording(const std::filesystem::path& path,
                                  const std::string& metadata) {
    if(path.empty()) {
        error_ = "Choose a session file path first.";
        return false;
    }
    if(recording_) stopRecording();
    closeReplay();
    closeDatabase();
    error_.clear();
    storageFailed_ = false;

    std::error_code directoryError;
    if(!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), directoryError);
        if(directoryError) {
            error_ = "Could not create the session directory: " +
                     directoryError.message();
            return false;
        }
    }

    if(std::filesystem::exists(path, directoryError) && !directoryError &&
       std::filesystem::file_size(path, directoryError) > 0 && !directoryError) {
        error_ = "Session file already exists. Use a new file for each recording.";
        return false;
    }

    path_ = path;
    const int openResult = sqlite3_open16(path_.c_str(), &database_);
    if(openResult != SQLITE_OK) {
        error_ = database_ != nullptr ? sqlite3_errmsg(database_)
                                      : "SQLite could not open the session file.";
        closeDatabase();
        return false;
    }
    sqlite3_busy_timeout(database_, 1500);
    if(!execute("PRAGMA journal_mode=WAL; PRAGMA synchronous=NORMAL;"
                "CREATE TABLE IF NOT EXISTS sessions("
                "id INTEGER PRIMARY KEY, opened_utc TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,"
                "protocol_version INTEGER NOT NULL, metadata TEXT NOT NULL,"
                "complete INTEGER NOT NULL DEFAULT 0, duration_ms INTEGER NOT NULL DEFAULT 0);"
                "CREATE TABLE IF NOT EXISTS packets("
                "id INTEGER PRIMARY KEY, session_id INTEGER NOT NULL,"
                "elapsed_ms INTEGER NOT NULL, direction INTEGER NOT NULL,"
                "peer_ip TEXT NOT NULL, peer_port INTEGER NOT NULL, datagram BLOB NOT NULL,"
                "timeline_order INTEGER NOT NULL,"
                "FOREIGN KEY(session_id) REFERENCES sessions(id));"
                "CREATE INDEX IF NOT EXISTS packets_by_time ON packets(session_id, direction, elapsed_ms, id);"
                "CREATE TABLE IF NOT EXISTS events("
                "id INTEGER PRIMARY KEY, session_id INTEGER NOT NULL,"
                "elapsed_ms INTEGER NOT NULL, kind TEXT NOT NULL, detail TEXT NOT NULL,"
                "timeline_order INTEGER NOT NULL,"
                "FOREIGN KEY(session_id) REFERENCES sessions(id));"
                "PRAGMA user_version=2;")) {
        closeDatabase();
        return false;
    }

    sqlite3_stmt* createSession = nullptr;
    if(sqlite3_prepare_v2(database_,
                          "INSERT INTO sessions(protocol_version, metadata) VALUES(?1, ?2)",
                          -1, &createSession, nullptr) != SQLITE_OK) {
        error_ = sqlite3_errmsg(database_);
        closeDatabase();
        return false;
    }
    sqlite3_bind_int(createSession, 1, PROTOCOL_VERSION);
    bindTransientText(createSession, 2, metadata);
    const int insertResult = sqlite3_step(createSession);
    sqlite3_finalize(createSession);
    if(insertResult != SQLITE_DONE || !prepareStatements() ||
       !execute("BEGIN IMMEDIATE")) {
        if(error_.empty()) error_ = sqlite3_errmsg(database_);
        closeDatabase();
        return false;
    }

    recordingStartedAt_ = std::chrono::steady_clock::now();
    pausedDuration_ = std::chrono::milliseconds(0);
    paused_ = false;
    recording_ = true;
    replaying_ = false;
    writesSinceCommit_ = 0;
    nextTimelineOrder_ = 1;
    if(!insertEvent("recording_started", pathToUtf8(path_))) {
        rollbackFailedRecording();
        return false;
    }
    return true;
}

bool SessionStore::pauseRecording() {
    if(!recording_ || paused_ || storageFailed_) return false;
    pausedAt_ = std::chrono::steady_clock::now();
    if(!insertEvent("recording_paused", "Capture paused from desktop.") ||
       !flushBatch(false)) return false;
    paused_ = true;
    return true;
}

bool SessionStore::resumeRecording() {
    if(!recording_ || !paused_ || storageFailed_) return false;
    if(!execute("BEGIN IMMEDIATE")) {
        storageFailed_ = true;
        return false;
    }
    pausedDuration_ += std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - pausedAt_);
    paused_ = false;
    writesSinceCommit_ = 0;
    return insertEvent("recording_resumed", "Capture resumed from desktop.");
}

bool SessionStore::stopRecording() {
    if(!recording_) return true;
    if(storageFailed_) {
        rollbackFailedRecording();
        return false;
    }
    if(paused_) {
        pausedDuration_ += std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - pausedAt_);
        paused_ = false;
        if(!execute("BEGIN IMMEDIATE")) {
            storageFailed_ = true;
            rollbackFailedRecording();
            return false;
        }
    }
    if(!insertEvent("recording_stopped", "Capture stopped from desktop.")) {
        rollbackFailedRecording();
        return false;
    }
    const std::uint64_t duration = elapsedRecordingMilliseconds();
    sqlite3_stmt* update = nullptr;
    if(sqlite3_prepare_v2(database_,
                          "UPDATE sessions SET complete=1, duration_ms=?1 WHERE id=(SELECT MAX(id) FROM sessions)",
                          -1, &update, nullptr) != SQLITE_OK) {
        error_ = sqlite3_errmsg(database_);
        storageFailed_ = true;
        rollbackFailedRecording();
        return false;
    }
    sqlite3_bind_int64(update, 1, static_cast<sqlite3_int64>(duration));
    const int updateResult = sqlite3_step(update);
    sqlite3_finalize(update);
    if(updateResult != SQLITE_DONE) {
        error_ = sqlite3_errmsg(database_);
        storageFailed_ = true;
        rollbackFailedRecording();
        return false;
    }
    if(!flushBatch(false)) {
        storageFailed_ = true;
        rollbackFailedRecording();
        return false;
    }
    recording_ = false;
    replaying_ = false;
    execute("PRAGMA wal_checkpoint(TRUNCATE)");
    return true;
}

void SessionStore::recordPacket(const Packet& packet, const PeerAddress& peer,
                                bool outgoing) {
    if(!isRecording() || storageFailed_ || packet.size == 0 || packet.size > BUFFER_LEN ||
       packet.size > static_cast<std::uint16_t>(maximumDatagramBytes) ||
       packetInsert_ == nullptr) return;

    const std::string peerIp(peer.ip,
        std::find(std::begin(peer.ip), std::end(peer.ip), '\0'));
    sqlite3_bind_int64(packetInsert_, 1, static_cast<sqlite3_int64>(
        elapsedRecordingMilliseconds()));
    sqlite3_bind_int(packetInsert_, 2, outgoing ? 1 : 0);
    bindTransientText(packetInsert_, 3, peerIp);
    sqlite3_bind_int(packetInsert_, 4, peer.port);
    sqlite3_bind_blob(packetInsert_, 5, packet.buffer, packet.size, SQLITE_TRANSIENT);
    sqlite3_bind_int64(packetInsert_, 6,
                       static_cast<sqlite3_int64>(nextTimelineOrder_++));
    const int result = sqlite3_step(packetInsert_);
    sqlite3_reset(packetInsert_);
    sqlite3_clear_bindings(packetInsert_);
    if(result != SQLITE_DONE) {
        error_ = sqlite3_errmsg(database_);
        storageFailed_ = true;
        return;
    }
    if(++writesSinceCommit_ >= commitBatchSize && !flushBatch(true)) {
        storageFailed_ = true;
    }
}

void SessionStore::recordEvent(const std::string& kind,
                               const std::string& detail) {
    if(!recording_ || storageFailed_) return;
    insertEvent(kind, detail);
}

bool SessionStore::openReplay(const std::filesystem::path& path) {
    stopRecording();
    closeReplay();
    closeDatabase();
    error_.clear();
    replayMetadata_.clear();
    path_ = path;
    std::error_code pathError;
    if(!std::filesystem::exists(path_, pathError) || pathError) {
        error_ = "Session file does not exist.";
        return false;
    }
    if(sqlite3_open16(path_.c_str(), &database_) != SQLITE_OK) {
        error_ = database_ != nullptr ? sqlite3_errmsg(database_)
                                      : "SQLite could not open the replay file.";
        closeDatabase();
        return false;
    }
    sqlite3_busy_timeout(database_, 1500);
    storageFailed_ = false;

    sqlite3_stmt* schemaVersionQuery = nullptr;
    if(sqlite3_prepare_v2(database_, "PRAGMA user_version", -1,
                          &schemaVersionQuery, nullptr) != SQLITE_OK ||
       sqlite3_step(schemaVersionQuery) != SQLITE_ROW) {
        error_ = sqlite3_errmsg(database_);
        if(schemaVersionQuery != nullptr) sqlite3_finalize(schemaVersionQuery);
        closeDatabase();
        return false;
    }
    const int schemaVersion = sqlite3_column_int(schemaVersionQuery, 0);
    sqlite3_finalize(schemaVersionQuery);
    // Earlier ThermalRover session files had the same v1 schema but did not
    // set user_version. Keep them replayable while rejecting future schemas
    // whose tables or packet interpretation may be incompatible.
    if(schemaVersion < 0 || schemaVersion > currentSessionSchemaVersion) {
        error_ = "This session uses an unsupported database schema version (" +
                 std::to_string(schemaVersion) + ").";
        closeDatabase();
        return false;
    }
    replayHasTimelineOrder_ = schemaVersion >= 2;

    sqlite3_stmt* session = nullptr;
    if(sqlite3_prepare_v2(database_,
                          "SELECT id, protocol_version, complete, MAX(duration_ms,"
                          "COALESCE((SELECT MAX(elapsed_ms) FROM packets WHERE session_id=sessions.id),0),"
                          "COALESCE((SELECT MAX(elapsed_ms) FROM events WHERE session_id=sessions.id),0)), metadata "
                          "FROM sessions ORDER BY id DESC LIMIT 1",
                          -1, &session, nullptr) != SQLITE_OK ||
       sqlite3_step(session) != SQLITE_ROW) {
        error_ = sqlite3_errmsg(database_);
        if(session != nullptr) sqlite3_finalize(session);
        closeDatabase();
        return false;
    }
    const sqlite3_int64 sessionId = sqlite3_column_int64(session, 0);
    const int sessionProtocolVersion = sqlite3_column_int(session, 1);
    if(sessionId <= 0) {
        error_ = "This session has an invalid database session ID.";
        sqlite3_finalize(session);
        closeDatabase();
        return false;
    }
    if(sessionProtocolVersion != PROTOCOL_VERSION &&
       sessionProtocolVersion != PREVIOUS_REPLAY_PROTOCOL_VERSION &&
       sessionProtocolVersion != LEGACY_REPLAY_PROTOCOL_VERSION) {
        error_ = "This session uses an unsupported rover protocol version.";
        sqlite3_finalize(session);
        closeDatabase();
        return false;
    }
    replaySessionId_ = sessionId;
    replayProtocolVersion_ = sessionProtocolVersion;
    replayComplete_ = sqlite3_column_int(session, 2) != 0;
    replayDurationMs_ = static_cast<std::uint64_t>(
        std::max<sqlite3_int64>(0, sqlite3_column_int64(session, 3)));
    const auto* metadata = reinterpret_cast<const char*>(
        sqlite3_column_text(session, 4));
    const int metadataBytes = sqlite3_column_bytes(session, 4);
    if(metadata != nullptr && metadataBytes > 0) {
        replayMetadata_.assign(metadata, static_cast<std::size_t>(metadataBytes));
    }
    sqlite3_finalize(session);
    replaying_ = true;
    if(!rewindReplayPackets() || !rewindReplayEvents()) {
        replaying_ = false;
        closeDatabase();
        return false;
    }
    return true;
}

bool SessionStore::rewindReplayPackets() {
    if(!replaying_ || database_ == nullptr) return false;
    replayPacketsExhausted_ = true;
    if(replayQuery_ != nullptr) {
        sqlite3_finalize(replayQuery_);
        replayQuery_ = nullptr;
    }
    // Replay always starts at the first rover packet. The application resets its
    // mapper and advances this cursor up to the scrub target so the displayed
    // map represents all observations before that time.
    const char* query = replayHasTimelineOrder_
        ? "SELECT id, elapsed_ms, peer_ip, peer_port, datagram, timeline_order FROM packets "
          "WHERE session_id=?1 AND direction=0 ORDER BY elapsed_ms, timeline_order, id"
        : "SELECT id, elapsed_ms, peer_ip, peer_port, datagram, 0 FROM packets "
          "WHERE session_id=?1 AND direction=0 ORDER BY elapsed_ms, id";
    if(sqlite3_prepare_v2(database_, query, -1, &replayQuery_, nullptr) != SQLITE_OK ||
       sqlite3_bind_int64(replayQuery_, 1, replaySessionId_) != SQLITE_OK) {
        error_ = sqlite3_errmsg(database_);
        sqlite3_finalize(replayQuery_);
        replayQuery_ = nullptr;
        return false;
    }
    replayPacketsExhausted_ = false;
    return true;
}

bool SessionStore::rewindReplayEvents() {
    if(!replaying_ || database_ == nullptr) return false;
    replayEventsExhausted_ = true;
    if(replayEventQuery_ != nullptr) {
        sqlite3_finalize(replayEventQuery_);
        replayEventQuery_ = nullptr;
    }
    const char* query = replayHasTimelineOrder_
        ? "SELECT id, elapsed_ms, kind, detail, timeline_order FROM events "
          "WHERE session_id=?1 ORDER BY elapsed_ms, timeline_order, id"
        : "SELECT id, elapsed_ms, kind, detail, 1 FROM events "
          "WHERE session_id=?1 ORDER BY elapsed_ms, id";
    if(sqlite3_prepare_v2(database_, query, -1, &replayEventQuery_, nullptr) !=
           SQLITE_OK ||
       sqlite3_bind_int64(replayEventQuery_, 1, replaySessionId_) != SQLITE_OK) {
        error_ = sqlite3_errmsg(database_);
        sqlite3_finalize(replayEventQuery_);
        replayEventQuery_ = nullptr;
        return false;
    }
    replayEventsExhausted_ = false;
    return true;
}

bool SessionStore::readReplayPacket(Packet& packet, PeerAddress& peer,
                                    std::uint64_t& elapsedMilliseconds,
                                    std::uint64_t* timelineOrder) {
    if(!replaying_ || replayQuery_ == nullptr || replayPacketsExhausted_) return false;
    // sqlite3_step may automatically restart a statement after SQLITE_DONE.
    // Only an explicit rewind is allowed to restart the session cursor.
    replayPacketsExhausted_ = true;
    const int result = sqlite3_step(replayQuery_);
    if(result == SQLITE_DONE) return false;
    if(result != SQLITE_ROW) {
        error_ = sqlite3_errmsg(database_);
        return false;
    }
    const sqlite3_int64 packetRow = sqlite3_column_int64(replayQuery_, 0);
    const sqlite3_int64 elapsed = sqlite3_column_int64(replayQuery_, 1);
    if(elapsed < 0) {
        error_ = "Session replay stopped at packet row " +
                 std::to_string(packetRow) + ": stored packet time is invalid.";
        return false;
    }
    elapsedMilliseconds = static_cast<std::uint64_t>(elapsed);
    const sqlite3_int64 order = sqlite3_column_int64(replayQuery_, 5);
    if(order < 0 || (replayHasTimelineOrder_ && order == 0)) {
        error_ = "Session replay stopped at packet row " +
                 std::to_string(packetRow) + ": stored timeline order is invalid.";
        return false;
    }
    if(timelineOrder != nullptr) *timelineOrder = static_cast<std::uint64_t>(order);
    peer = {};
    const auto* ip = reinterpret_cast<const char*>(sqlite3_column_text(replayQuery_, 2));
    const int ipLength = sqlite3_column_bytes(replayQuery_, 2);
    if(ip != nullptr && ipLength > 0) {
        const std::size_t copyLength = std::min<std::size_t>(
            static_cast<std::size_t>(ipLength), sizeof(peer.ip) - 1);
        std::copy_n(ip, copyLength, peer.ip);
        peer.ip[copyLength] = '\0';
    }
    const int peerPort = sqlite3_column_int(replayQuery_, 3);
    if(peerPort < 0 || peerPort > std::numeric_limits<unsigned short>::max()) {
        error_ = "Session replay stopped at packet row " + std::to_string(packetRow) +
                 ": stored peer port is invalid.";
        return false;
    }
    peer.port = static_cast<unsigned short>(peerPort);
    const auto* blob = static_cast<const std::uint8_t*>(
        sqlite3_column_blob(replayQuery_, 4));
    const int blobBytes = sqlite3_column_bytes(replayQuery_, 4);
    const std::size_t storedHeaderSize = replayProtocolVersion_ ==
        LEGACY_REPLAY_PROTOCOL_VERSION
        ? sizeof(LegacyPacketHeaderV1) : sizeof(PacketHeader);
    if(blob == nullptr || blobBytes < static_cast<int>(storedHeaderSize) ||
       blobBytes > maximumDatagramBytes) {
        error_ = "Session replay stopped at packet row " + std::to_string(packetRow) +
                 ": stored datagram size is invalid.";
        return false;
    }

    if(replayProtocolVersion_ == LEGACY_REPLAY_PROTOCOL_VERSION) {
        LegacyPacketHeaderV1 legacyHeader{};
        std::memcpy(&legacyHeader, blob, sizeof(legacyHeader));
        if(sizeof(LegacyPacketHeaderV1) + legacyHeader.length !=
               static_cast<std::size_t>(blobBytes) ||
           sizeof(PacketHeader) + legacyHeader.length > BUFFER_LEN) {
            error_ = "Session replay stopped at packet row " + std::to_string(packetRow) +
                     ": stored legacy datagram has an invalid header or length.";
            return false;
        }

        PacketHeader normalizedHeader{};
        normalizedHeader.length = legacyHeader.length;
        normalizedHeader.type = legacyHeader.type;
        normalizedHeader.id = legacyHeader.id;
        normalizedHeader.seq = legacyHeader.seq;
        normalizedHeader.version = LEGACY_REPLAY_PROTOCOL_VERSION;
        packet = Packet();
        std::memcpy(packet.buffer, &normalizedHeader, sizeof(normalizedHeader));
        std::memcpy(packet.buffer + sizeof(normalizedHeader),
                    blob + sizeof(LegacyPacketHeaderV1), legacyHeader.length);
        packet.size = static_cast<std::uint16_t>(
            sizeof(normalizedHeader) + legacyHeader.length);
        replayPacketsExhausted_ = false;
        return true;
    }

    PacketHeader header{};
    std::memcpy(&header, blob, sizeof(header));
    if(header.version != replayProtocolVersion_ ||
       sizeof(PacketHeader) + header.length != static_cast<std::size_t>(blobBytes)) {
        error_ = "Session replay stopped at packet row " + std::to_string(packetRow) +
                 ": stored datagram has an invalid protocol header or length.";
        return false;
    }
    packet = Packet(const_cast<std::uint8_t*>(blob), static_cast<std::uint16_t>(blobBytes));
    replayPacketsExhausted_ = false;
    return true;
}

bool SessionStore::readReplayEvent(std::string& kind, std::string& detail,
                                   std::uint64_t& elapsedMilliseconds,
                                   std::uint64_t* timelineOrder) {
    if(!replaying_ || replayEventQuery_ == nullptr || replayEventsExhausted_) return false;
    replayEventsExhausted_ = true;
    const int result = sqlite3_step(replayEventQuery_);
    if(result == SQLITE_DONE) return false;
    if(result != SQLITE_ROW) {
        error_ = sqlite3_errmsg(database_);
        return false;
    }

    const sqlite3_int64 eventRow = sqlite3_column_int64(replayEventQuery_, 0);
    const sqlite3_int64 elapsed = sqlite3_column_int64(replayEventQuery_, 1);
    if(elapsed < 0) {
        error_ = "Session replay stopped at event row " +
                 std::to_string(eventRow) + ": stored event time is invalid.";
        return false;
    }
    elapsedMilliseconds = static_cast<std::uint64_t>(elapsed);
    const sqlite3_int64 order = sqlite3_column_int64(replayEventQuery_, 4);
    if(order < 0 || (replayHasTimelineOrder_ && order == 0)) {
        error_ = "Session replay stopped at event row " +
                 std::to_string(eventRow) + ": stored timeline order is invalid.";
        return false;
    }
    if(timelineOrder != nullptr) *timelineOrder = static_cast<std::uint64_t>(order);

    const auto readText = [&](int column, std::string& value) {
        const auto* bytes = reinterpret_cast<const char*>(
            sqlite3_column_text(replayEventQuery_, column));
        const int byteCount = sqlite3_column_bytes(replayEventQuery_, column);
        if(bytes == nullptr || byteCount <= 0) {
            value.clear();
        } else {
            value.assign(bytes, static_cast<std::size_t>(byteCount));
        }
    };
    readText(2, kind);
    readText(3, detail);
    replayEventsExhausted_ = false;
    return true;
}

void SessionStore::closeReplay() {
    if(replayQuery_ != nullptr) {
        sqlite3_finalize(replayQuery_);
        replayQuery_ = nullptr;
    }
    if(replayEventQuery_ != nullptr) {
        sqlite3_finalize(replayEventQuery_);
        replayEventQuery_ = nullptr;
    }
    replaying_ = false;
    replayDurationMs_ = 0;
    replaySessionId_ = 0;
    replayComplete_ = false;
    replayHasTimelineOrder_ = false;
    replayPacketsExhausted_ = true;
    replayEventsExhausted_ = true;
    replayMetadata_.clear();
}

std::uint64_t SessionStore::elapsedRecordingMilliseconds() const {
    if(recordingStartedAt_.time_since_epoch().count() == 0) return 0;
    const auto now = paused_ ? pausedAt_ : std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - recordingStartedAt_) - pausedDuration_;
    return static_cast<std::uint64_t>(std::max<std::int64_t>(0, elapsed.count()));
}

bool SessionStore::execute(const char* sql) {
    if(database_ == nullptr) return false;
    char* message = nullptr;
    const int result = sqlite3_exec(database_, sql, nullptr, nullptr, &message);
    if(result == SQLITE_OK) return true;
    error_ = message != nullptr ? message : sqlite3_errmsg(database_);
    sqlite3_free(message);
    return false;
}

bool SessionStore::prepareStatements() {
    constexpr const char* packetSql =
        "INSERT INTO packets(session_id, elapsed_ms, direction, peer_ip, peer_port, datagram, timeline_order) "
        "VALUES((SELECT MAX(id) FROM sessions), ?1, ?2, ?3, ?4, ?5, ?6)";
    constexpr const char* eventSql =
        "INSERT INTO events(session_id, elapsed_ms, kind, detail, timeline_order) "
        "VALUES((SELECT MAX(id) FROM sessions), ?1, ?2, ?3, ?4)";
    if(sqlite3_prepare_v2(database_, packetSql, -1, &packetInsert_, nullptr) != SQLITE_OK ||
       sqlite3_prepare_v2(database_, eventSql, -1, &eventInsert_, nullptr) != SQLITE_OK) {
        error_ = sqlite3_errmsg(database_);
        finalizeStatements();
        return false;
    }
    return true;
}

bool SessionStore::flushBatch(bool continueTransaction) {
    if(database_ == nullptr) return false;
    if(!execute("COMMIT")) {
        storageFailed_ = true;
        return false;
    }
    writesSinceCommit_ = 0;
    if(continueTransaction && recording_ && !paused_) {
        if(!execute("BEGIN IMMEDIATE")) {
            storageFailed_ = true;
            return false;
        }
    }
    return true;
}

bool SessionStore::insertEvent(const std::string& kind,
                               const std::string& detail) {
    if(eventInsert_ == nullptr) return false;
    sqlite3_bind_int64(eventInsert_, 1, static_cast<sqlite3_int64>(
        elapsedRecordingMilliseconds()));
    bindTransientText(eventInsert_, 2, kind);
    bindTransientText(eventInsert_, 3, detail);
    sqlite3_bind_int64(eventInsert_, 4,
                       static_cast<sqlite3_int64>(nextTimelineOrder_++));
    const int result = sqlite3_step(eventInsert_);
    sqlite3_reset(eventInsert_);
    sqlite3_clear_bindings(eventInsert_);
    if(result != SQLITE_DONE) {
        error_ = sqlite3_errmsg(database_);
        storageFailed_ = true;
        return false;
    }
    // Paused captures have no open write transaction. SQLite commits each
    // event automatically; keep it out of the active batch counter so a long
    // pause with operator events cannot try to COMMIT a nonexistent batch.
    if(paused_) return true;
    if(++writesSinceCommit_ >= commitBatchSize && !flushBatch(true)) {
        storageFailed_ = true;
        return false;
    }
    return true;
}

void SessionStore::rollbackFailedRecording() {
    if(database_ != nullptr) {
        // Preserve the original storage error; a failed COMMIT may already have
        // ended the transaction, in which case ROLLBACK itself is expected to fail.
        sqlite3_exec(database_, "ROLLBACK", nullptr, nullptr, nullptr);
    }
    recording_ = false;
    paused_ = false;
    writesSinceCommit_ = 0;
}

void SessionStore::finalizeStatements() {
    if(packetInsert_ != nullptr) {
        sqlite3_finalize(packetInsert_);
        packetInsert_ = nullptr;
    }
    if(eventInsert_ != nullptr) {
        sqlite3_finalize(eventInsert_);
        eventInsert_ = nullptr;
    }
    if(replayQuery_ != nullptr) {
        sqlite3_finalize(replayQuery_);
        replayQuery_ = nullptr;
    }
    if(replayEventQuery_ != nullptr) {
        sqlite3_finalize(replayEventQuery_);
        replayEventQuery_ = nullptr;
    }
}

void SessionStore::closeDatabase() {
    finalizeStatements();
    if(database_ != nullptr) {
        sqlite3_close_v2(database_);
        database_ = nullptr;
    }
    recording_ = false;
    paused_ = false;
    replaying_ = false;
    replayProtocolVersion_ = 0;
    replaySessionId_ = 0;
}
