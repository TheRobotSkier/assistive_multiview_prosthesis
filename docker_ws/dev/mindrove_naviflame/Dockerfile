FROM python:3.10-slim

ENV DEBIAN_FRONTEND=noninteractive

# System libraries required by OpenCV HighGUI (X11 display) and the MindRove SDK
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libusb-1.0-0 \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
        libx11-6 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Pin numpy<1.24 (TensorFlow 2.12 is incompatible with numpy 1.24+ dtype changes)
# and scikit-learn to a version compatible with the pre-built .pkl model artifacts.
RUN pip install --no-cache-dir \
        "numpy==1.23.5" \
        "scikit-learn==1.2.2" \
        "tensorflow==2.12.0" \
        "opencv-python==4.8.0.76" \
        "mindrove" \
        "keyboard"

COPY NaviFlame/ /app/NaviFlame/

# docker-config.json overrides NaviFlame/config.json:
#   - show_predicted_image: true  → display gesture via OpenCV imshow (requires X11)
#   - send_to_socket: false       → Unity .exe visualiser cannot run on Linux
COPY docker-config.json /app/NaviFlame/config.json

WORKDIR /app/NaviFlame

CMD ["python", "examples/inference_example.py"]
