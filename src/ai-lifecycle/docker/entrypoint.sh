#!/usr/bin/env bash
# entrypoint.sh — Bootstrap the ai-lifecycle runtime environment.
#
# Steps:
#   1. Read AILC_CONFIG to find the experiment YAML.
#   2. Parse the dependencies.packages list using Python + PyYAML.
#   3. Hash the sorted package list (sha256) to derive ENV_HASH.
#   4. If /envs/$ENV_HASH does not exist, create a uv venv and install.
#   5. Export AILC_PYTHON pointing at the venv's Python binary.
#   6. If a stage argument (train/evaluate/infer) is given, run it;
#      otherwise exec "$@" or keep the container alive.

set -euo pipefail

AILC_CONFIG="${AILC_CONFIG:-/workspace/src/ai-lifecycle/configs/yolov11_detect.yaml}"
ENVS_ROOT="${ENVS_ROOT:-/envs}"

echo "[entrypoint] Config: ${AILC_CONFIG}"

# ---------------------------------------------------------------------------
# 1. Parse packages from YAML
# ---------------------------------------------------------------------------
PACKAGES_JSON="$(python3 -c "
import sys, json, yaml
config_path = sys.argv[1]
try:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    pkgs = cfg.get('dependencies', {}).get('packages', [])
    extra = cfg.get('dependencies', {}).get('extra_index_url') or ''
    python_ver = cfg.get('dependencies', {}).get('python', '3.11')
    print(json.dumps({'packages': pkgs, 'extra_index_url': extra, 'python': python_ver}))
except Exception as e:
    print(json.dumps({'packages': [], 'extra_index_url': '', 'python': '3.11'}))
    sys.stderr.write(f'[entrypoint] Warning: could not parse config: {e}\n')
" "${AILC_CONFIG}")"

PACKAGES="$(echo "${PACKAGES_JSON}" | python3 -c "import sys,json; d=json.load(sys.stdin); print('\n'.join(sorted(d['packages'])))")"
EXTRA_INDEX="$(echo "${PACKAGES_JSON}" | python3 -c "import sys,json; print(json.load(sys.stdin)['extra_index_url'])")"
PYTHON_VER="$(echo "${PACKAGES_JSON}" | python3 -c "import sys,json; print(json.load(sys.stdin)['python'])")"

# ---------------------------------------------------------------------------
# 2. Hash the sorted package list
# ---------------------------------------------------------------------------
ENV_HASH="$(echo "${PACKAGES}" | sha256sum | cut -c1-16)"
ENV_PATH="${ENVS_ROOT}/${ENV_HASH}"

echo "[entrypoint] Dependency hash: ${ENV_HASH}"
echo "[entrypoint] Env path:        ${ENV_PATH}"

# ---------------------------------------------------------------------------
# 3. Provision the environment if needed
# ---------------------------------------------------------------------------
if [ ! -f "${ENV_PATH}/bin/python" ]; then
    echo "[entrypoint] Provisioning new environment …"

    uv venv --python "${PYTHON_VER}" "${ENV_PATH}"

    if [ -n "${PACKAGES}" ]; then
        INSTALL_CMD="uv pip install --python ${ENV_PATH}/bin/python"
        if [ -n "${EXTRA_INDEX}" ] && [ "${EXTRA_INDEX}" != "None" ]; then
            INSTALL_CMD="${INSTALL_CMD} --extra-index-url ${EXTRA_INDEX}"
        fi
        # shellcheck disable=SC2086
        eval ${INSTALL_CMD} ${PACKAGES}
    fi

    echo "[entrypoint] Environment provisioned at ${ENV_PATH}"
else
    echo "[entrypoint] Reusing existing environment at ${ENV_PATH}"
fi

# ---------------------------------------------------------------------------
# 4. Export AILC_PYTHON
# ---------------------------------------------------------------------------
export AILC_PYTHON="${ENV_PATH}/bin/python"
echo "[entrypoint] AILC_PYTHON=${AILC_PYTHON}"

# ---------------------------------------------------------------------------
# 5. Dispatch or exec
# ---------------------------------------------------------------------------
STAGE="${1:-}"

case "${STAGE}" in
    train|evaluate|infer)
        echo "[entrypoint] Running stage: ${STAGE}"
        exec "${AILC_PYTHON}" -m ai_lifecycle.cli "${STAGE}" -c "${AILC_CONFIG}"
        ;;
    "")
        # No command — keep container alive (compose uses `sleep infinity`)
        exec "$@"
        ;;
    *)
        exec "$@"
        ;;
esac
