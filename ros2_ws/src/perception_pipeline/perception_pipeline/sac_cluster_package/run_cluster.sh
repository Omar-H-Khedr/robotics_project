#!/usr/bin/env bash
# =============================================================================
# run_cluster.sh - SLURM Job Submission Script
# =============================================================================
# Submits SAC training job to SLURM cluster.
#
# Usage:
#   sbatch run_cluster.sh                          # Default 1M steps
#   sbatch --export=TOTAL_STEPS=5000000 run_cluster.sh  # 5M steps
#   bash run_cluster.sh                             # Run locally (no SLURM)
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration (override via environment variables)
# ---------------------------------------------------------------------------
TOTAL_STEPS="${TOTAL_STEPS:-1000000}"
SCENARIO_MODE="${SCENARIO_MODE:-uniform}"
SEED="${SEED:-42}"
CHECKPOINT_DIR="${CHECKPOINT_DIR:-/scratch/${USER}/sac_checkpoints}"
LOG_DIR="${LOG_DIR:-/scratch/${USER}/sac_logs}"
JOB_NAME="${JOB_NAME:-sac_train}"
PARTITION="${PARTITION:-gpu}"
TIME_LIMIT="${TIME_LIMIT:-24:00:00}"
ACCOUNT="${ACCOUNT:-robotics}"

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# Environment Detection
# ---------------------------------------------------------------------------
IS_CLUSTER=false
if command -v sbatch &>/dev/null && [ -f "/etc/slurm" ] || [ -d "/opt/slurm" ] || [ -n "${SLURM_JOB_ID:-}" ]; then
    IS_CLUSTER=true
fi

# Also check for common cluster indicators
if [ -d "/scratch" ] || [ -d "/opt/slurm" ] || [ -n "${SBATCH:-}" ]; then
    IS_CLUSTER=true
fi

echo "============================================================"
echo "SAC Training - Cluster Submission Script"
echo "============================================================"
echo "  Steps:         ${TOTAL_STEPS}"
echo "  Scenario:      ${SCENARIO_MODE}"
echo "  Seed:          ${SEED}"
echo "  Checkpoint:    ${CHECKPOINT_DIR}"
echo "  Log:           ${LOG_DIR}"
echo "  Cluster mode:  ${IS_CLUSTER}"
echo "============================================================"

# ---------------------------------------------------------------------------
# Module Loading (cluster only)
# ---------------------------------------------------------------------------
if [ "$IS_CLUSTER" = true ]; then
    echo "[INFO] Loading cluster modules..."
    module purge 2>/dev/null || true

    # Load required modules (adjust for your cluster)
    module load python/3.10 2>/dev/null || echo "[WARN] python/3.10 module not available"
    module load cuda/11.8 2>/dev/null || echo "[WARN] cuda/11.8 module not available"

    echo "[INFO] Python: $(python3 --version 2>&1)"
    echo "[INFO] CUDA:   $(nvcc --version 2>/dev/null | tail -1 || echo 'not found')"
fi

# ---------------------------------------------------------------------------
# Virtual Environment
# ---------------------------------------------------------------------------
VENV_PATH="${VENV_PATH:-${SCRIPT_DIR}/../.venv}"
if [ -d "$VENV_PATH" ]; then
    echo "[INFO] Activating virtual environment: ${VENV_PATH}"
    source "${VENV_PATH}/bin/activate"
else
    echo "[WARN] Virtual environment not found at ${VENV_PATH}"
    echo "[WARN] Using system Python"
fi

# Verify packages
echo "[INFO] Checking required packages..."
python3 -c "import torch; print(f'  PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')" 2>/dev/null || {
    echo "[ERROR] PyTorch not available. Install: pip install torch"
    exit 1
}
python3 -c "import numpy; print(f'  NumPy: {numpy.__version__}')" 2>/dev/null || {
    echo "[ERROR] NumPy not available. Install: pip install numpy"
    exit 1
}

# ---------------------------------------------------------------------------
# Create output directories
# ---------------------------------------------------------------------------
mkdir -p "${CHECKPOINT_DIR}"
mkdir -p "${LOG_DIR}"

# ---------------------------------------------------------------------------
# Training Command
# ---------------------------------------------------------------------------
TRAINING_CMD="python3 ${SCRIPT_DIR}/sac_training_script.py \
    --total_steps ${TOTAL_STEPS} \
    --scenario_mode ${SCENARIO_MODE} \
    --seed ${SEED} \
    --checkpoint_dir ${CHECKPOINT_DIR} \
    --log_dir ${LOG_DIR} \
    --config ${SCRIPT_DIR}/config/cluster_training_config.yaml"

# Evaluation command (runs after training)
EVALUATION_CMD="python3 ${SCRIPT_DIR}/sac_evaluation_script.py \
    --checkpoint ${CHECKPOINT_DIR}/sac_final.pt \
    --episodes 100 \
    --held_out_scenarios \
    --output ${CHECKPOINT_DIR}/eval_report.json"

# ---------------------------------------------------------------------------
# Run on Cluster (SLURM)
# ---------------------------------------------------------------------------
if [ "$IS_CLUSTER" = true ]; then
    echo "[INFO] Submitting SLURM job..."

    SBATCH_CMD="sbatch \
        --job-name=${JOB_NAME} \
        --partition=${PARTITION} \
        --account=${ACCOUNT} \
        --time=${TIME_LIMIT} \
        --nodes=1 \
        --ntasks-per-node=1 \
        --cpus-per-task=8 \
        --gres=gpu:a100:1 \
        --mem=32G \
        --output=${LOG_DIR}/slurm-%j.out \
        --error=${LOG_DIR}/slurm-%j.err \
        --wrap=\"${TRAINING_CMD} && ${EVALUATION_CMD}\""

    eval "$SBATCH_CMD"

# ---------------------------------------------------------------------------
# Run Locally
# ---------------------------------------------------------------------------
else
    echo "[INFO] Running locally (no SLURM detected)..."
    echo "[INFO] Training command: ${TRAINING_CMD}"
    echo ""

    # Run training
    eval "${TRAINING_CMD}"

    # Run evaluation
    echo ""
    echo "[INFO] Running evaluation..."
    eval "${EVALUATION_CMD}"

    echo ""
    echo "[DONE] Training and evaluation complete."
fi
