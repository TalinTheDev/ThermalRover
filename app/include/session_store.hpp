#pragma once

#include "protocol/udp.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <chrono>

#include <winsqlite/winsqlite3.h>

// Each session is a self-contained SQLite file. Raw UDP datagrams are kept as
// received so that replay uses the same protocol decoder as live mapping.
class SessionStore {
public:
    SessionStore() = default;
    ~SessionStore();

    SessionStore(const SessionStore&) = delete;
    SessionStore& operator=(const SessionStore&) = delete;

    bool startRecording(const std::filesystem::path& path,
                        const std::string& metadata);
    bool pauseRecording();
    bool resumeRecording();
    bool stopRecording();
    void recordPacket(const Packet& packet, const PeerAddress& peer,
                      bool outgoing);
    void recordEvent(const std::string& kind, const std::string& detail);

    bool openReplay(const std::filesystem::path& path);
    bool rewindReplayPackets();
    bool rewindReplayEvents();
    bool readReplayPacket(Packet& packet, PeerAddress& peer,
                          std::uint64_t& elapsedMilliseconds,
                          std::uint64_t* timelineOrder = nullptr);
    bool readReplayEvent(std::string& kind, std::string& detail,
                         std::uint64_t& elapsedMilliseconds,
                         std::uint64_t* timelineOrder = nullptr);
    void closeReplay();
    const std::string& replayMetadata() const { return replayMetadata_; }

    bool isRecording() const { return recording_ && !paused_; }
    bool isPaused() const { return recording_ && paused_; }
    bool hasStorageFailure() const { return storageFailed_; }
    bool isReplaying() const { return replaying_; }
    bool hasDatabase() const { return database_ != nullptr; }
    bool replayIsComplete() const { return replayComplete_; }
    std::uint64_t recordingDurationMilliseconds() const {
        return recording_ ? elapsedRecordingMilliseconds() : 0;
    }
    std::uint64_t replayDurationMilliseconds() const { return replayDurationMs_; }
    const std::string& lastError() const { return error_; }
    const std::filesystem::path& path() const { return path_; }

private:
    sqlite3* database_ = nullptr;
    sqlite3_stmt* packetInsert_ = nullptr;
    sqlite3_stmt* eventInsert_ = nullptr;
    sqlite3_stmt* replayQuery_ = nullptr;
    sqlite3_stmt* replayEventQuery_ = nullptr;
    std::filesystem::path path_;
    std::string error_;
    std::string replayMetadata_;
    std::chrono::steady_clock::time_point recordingStartedAt_{};
    std::chrono::steady_clock::time_point pausedAt_{};
    std::chrono::milliseconds pausedDuration_{0};
    std::uint64_t replayDurationMs_ = 0;
    sqlite3_int64 replaySessionId_ = 0;
    std::size_t writesSinceCommit_ = 0;
    std::uint64_t nextTimelineOrder_ = 1;
    bool replayComplete_ = false;
    bool replayHasTimelineOrder_ = false;
    bool replayPacketsExhausted_ = true;
    bool replayEventsExhausted_ = true;
    bool storageFailed_ = false;
    int replayProtocolVersion_ = 0;
    bool recording_ = false;
    bool paused_ = false;
    bool replaying_ = false;

    std::uint64_t elapsedRecordingMilliseconds() const;
    bool execute(const char* sql);
    bool prepareStatements();
    bool flushBatch(bool continueTransaction);
    bool insertEvent(const std::string& kind, const std::string& detail);
    void rollbackFailedRecording();
    void finalizeStatements();
    void closeDatabase();
};
