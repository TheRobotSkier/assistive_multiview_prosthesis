# syntax=docker/dockerfile:1
FROM docker.io/python:3.12-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        libusb-1.0-0

RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages \
        mindrove \
        numpy \
        scipy \
        scikit-learn \
        joblib

WORKDIR /app

# Legacy reader — kept for reference
COPY read_emg.py /app/read_emg.py

# Classical EMG classifier package and scripts
COPY emg_classifier/ /app/emg_classifier/
COPY scripts/ /app/scripts/

# data/ and models/ are bind-mounted at runtime; create empty placeholders
RUN mkdir -p /app/data /app/models

CMD ["python", "scripts/run_classifier.py"]
