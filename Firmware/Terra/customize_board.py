Import("env")

# Add the custom variants path to the board configuration
board_config = env.BoardConfig()
board_config.update("build.variants_dir", "variants")