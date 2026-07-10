-- Configures neovim for this specific project

return {
	-- https://mason-registry.dev/registry/list (LSPs only)
	lsp = {
		clangd = {},
		cmakelang = {},
		glsl_analyzer = {},
	},

	-- https://mason-registry.dev/registry/list (Linters, Formatters, etc.)
	mason = {
		"clang-format",
		"cmakelang",
	},

	-- https://github.com/nvim-treesitter/nvim-treesitter?tab=readme-ov-file#supported-languages
	treesitter = {
		"cmake",
		"cpp",
		"dockerfile",
		"glsl",
	},
}
