#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "$repo_root"

# Deprecated runtime interface/contracts that must not reappear.
pattern='/grasp_preshaping/trigger|/grasp_preshaping/result|cargo run --release --features ros --bin ros_node'

if command -v rg >/dev/null 2>&1; then
  exclude_globs=(
    '--glob=!**/target/**'
    '--glob=!**/build/**'
    '--glob=!**/install/**'
    '--glob=!**/log/**'
    '--glob=!**/.git/**'
    '--glob=!**/.kilo/**'
    '--glob=!docker_ws/dev/grasp_preshaping/scripts/check_runtime_drift.sh'
  )

  if rg -n --no-heading -S -g'*.{cpp,hpp,c,cc,hh,py,rs,xml,yaml,yml,toml,md,txt,sh,launch}' "${exclude_globs[@]}" "$pattern" docker_ws README.md; then
    echo ""
    echo "[FAIL] Deprecated preshaping runtime references found."
    exit 1
  fi
else
  mapfile -t files < <(
    find docker_ws -type f \
      \( -name '*.cpp' -o -name '*.hpp' -o -name '*.c' -o -name '*.cc' -o -name '*.hh' -o -name '*.py' -o -name '*.rs' -o -name '*.xml' -o -name '*.yaml' -o -name '*.yml' -o -name '*.toml' -o -name '*.md' -o -name '*.txt' -o -name '*.sh' -o -name '*.launch' \) \
      ! -path '*/target/*' \
      ! -path '*/build/*' \
      ! -path '*/install/*' \
      ! -path '*/log/*' \
      ! -path '*/.git/*' \
        ! -path '*/.kilo/*' \
        ! -path '*/check_runtime_drift.sh'
  )

  if [[ -f README.md ]]; then
    files+=("README.md")
  fi

  if [[ ${#files[@]} -gt 0 ]] && grep -EnH "$pattern" "${files[@]}"; then
    echo ""
    echo "[FAIL] Deprecated preshaping runtime references found."
    exit 1
  fi
fi

echo "[PASS] No deprecated preshaping runtime references found in source/docs."
