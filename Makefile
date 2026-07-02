run:
	cargo run --release -F ssl_game_controller -F interface -F tracked_packages_check

run-debug:
	cargo run --release -F ssl_game_controller -F interface -F tracked_packages_check -F debug