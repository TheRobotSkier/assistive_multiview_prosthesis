# Prosthesis — common commands
# Works with podman-compose or docker compose (auto-detected)

COMPOSE_DIR := docker

# Auto-detect compose command
PODMAN_COMPOSE := $(shell which podman-compose 2>/dev/null)
DOCKER_COMPOSE := $(shell which docker 2>/dev/null)

ifdef PODMAN_COMPOSE
  COMPOSE := podman-compose
else
  COMPOSE := docker compose
endif

.PHONY: build build-prosthesis build-segmentation rebuild up up-hw up-grasp-test down-grasp-test logs-grasp-test up-digital-twin down-digital-twin logs-digital-twin test-digital-twin test shell clean logs

# ── Build ──────────────────────────────────────────────────────────────────
build:
	cd $(COMPOSE_DIR) && $(COMPOSE) build

build-prosthesis:
	cd $(COMPOSE_DIR) && $(COMPOSE) build prosthesis

build-segmentation:
	cd $(COMPOSE_DIR) && $(COMPOSE) build segmentation

rebuild:
	cd $(COMPOSE_DIR) && $(COMPOSE) build --no-cache

# ── Run ────────────────────────────────────────────────────────────────────
up:
	cd $(COMPOSE_DIR) && $(COMPOSE) up -d

up-hw:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile hardware up -d

# ── Grasp Test ──────────────────────────────────────────────────────────────
up-grasp-test:
	$(COMPOSE) rm -f grasp_test 2>/dev/null || true
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test up grasp_test -d

down-grasp-test:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test down

logs-grasp-test:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test logs -f

# ── Digital Twin ────────────────────────────────────────────────────────────
up-digital-twin:
	$(COMPOSE) rm -f digital_twin 2>/dev/null || true
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile digital_twin up digital_twin -d

down-digital-twin:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile digital_twin down

logs-digital-twin:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile digital_twin logs -f

test-digital-twin:
	@echo "=== Pointcloud Health Test ==="
	$(COMPOSE) exec digital_twin bash /prosthesis_ws/scripts/test_pointcloud_health.sh || \
		$(COMPOSE) exec grasp_test bash /prosthesis_ws/scripts/test_pointcloud_health.sh || \
		echo "No running container found — start one first with 'make up-digital-twin' or 'make up-grasp-test'"

# ── Test ───────────────────────────────────────────────────────────────────
test:
	cd $(COMPOSE_DIR) && $(COMPOSE) build prosthesis && $(COMPOSE) run --rm test

# ── Shell into running container ──────────────────────────────────────────
shell:
	cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash

# ── Cleanup ───────────────────────────────────────────────────────────────
down:
	cd $(COMPOSE_DIR) && $(COMPOSE) down

clean:
	cd $(COMPOSE_DIR) && $(COMPOSE) down --rmi local --volumes

logs:
	cd $(COMPOSE_DIR) && $(COMPOSE) logs -f
