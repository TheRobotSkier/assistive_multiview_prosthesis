FROM python:3.12-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libusb-1.0-0 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
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
