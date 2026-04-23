"""
Read EMG values from a MindRove WiFi board and print them to stdout and a log file.

Mirrors the official plot_real_time_min.py example (without Qt):
  https://github.com/MindRove/MindRoveSDK/blob/main/examples/python/plot_real_time/plot_real_time_min.py

Prerequisites:
  - MindRove armband powered on, host connected to its WiFi (192.168.4.x).
  - Container started with --network host (see docker-compose).
"""

import logging
import time

from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds

LOG_FILE = "/app/emg_log.txt"
UPDATE_SPEED_S = 0.05  # 50 ms — matches the Qt timer in the official example
WINDOW_SIZE_S  = 4     # seconds — same default as the official plot example


def main():
    BoardShim.enable_dev_board_logger()
    logging.basicConfig(level=logging.DEBUG)

    params = MindRoveInputParams()
    board_shim = None

    with open(LOG_FILE, "w") as lf:
        def log(msg):
            print(msg, flush=True)
            lf.write(msg + "\n")
            lf.flush()

        try:
            # --- connect (exactly as in plot_real_time_min.py main()) ---
            board_shim = BoardShim(BoardIds.MINDROVE_WIFI_BOARD, params)
            board_shim.prepare_session()
            board_shim.start_stream()

            # Use get_board_id() to obtain the integer id for static helpers,
            # mirroring Graph.__init__ in the official example.
            board_id      = board_shim.get_board_id()
            exg_channels  = BoardShim.get_exg_channels(board_id)
            sampling_rate = BoardShim.get_sampling_rate(board_id)
            num_points    = WINDOW_SIZE_S * sampling_rate  # e.g. 4 * 500 = 2000

            log(f"Connected!  board_id={board_id}  rate={sampling_rate} Hz  "
                f"channels={exg_channels}  window={num_points} samples")
            log("Polling every 50 ms — press Ctrl+C to stop.\n")

            # --- poll loop (mirrors Graph.update() called by Qt timer) ---
            while True:
                time.sleep(UPDATE_SPEED_S)

                # Exact same call as in the official update() method
                data = board_shim.get_current_board_data(num_points)

                if data.shape[1] == 0:
                    log("No samples in buffer yet...")
                    continue

                # Print one summary line per channel (raw mean over the window)
                parts = [f"ch{ch}={data[ch].mean():+.1f}" for ch in exg_channels]
                log(f"[{data.shape[1]:4d} smp]  " + "  ".join(parts))

        except KeyboardInterrupt:
            log("\nStopped by user.")
        except Exception:
            logging.warning("Exception", exc_info=True)
        finally:
            logging.info("End")
            if board_shim is not None and board_shim.is_prepared():
                board_shim.release_session()
                log("Session released.")


if __name__ == "__main__":
    main()
