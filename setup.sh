#!/usr/bin/env bash
# setup.sh – nastaví prostředí pro SALT Manipulator
# Spustit jako: bash setup.sh
# Na Raspberry Pi s hardwarem: bash setup.sh --hardware

set -euo pipefail

VENV_DIR=".venv"
INSTALL_HARDWARE=false

# ── Argumenty ─────────────────────────────────────────────────────────────────
for arg in "$@"; do
  case $arg in
    --hardware) INSTALL_HARDWARE=true ;;
    --help)
      echo "Použití: bash setup.sh [--hardware]"
      echo "  --hardware   Nainstaluje i knihovny Sequent Microsystems (vyžaduje RPi + sudo)"
      exit 0
      ;;
    *) echo "Neznámý argument: $arg"; exit 1 ;;
  esac
done

echo "======================================"
echo "  SALT Manipulator – setup"
echo "======================================"

# ── Python virtuální prostředí ─────────────────────────────────────────────────
echo ""
echo "[1/3] Vytvářím virtuální prostředí v $VENV_DIR ..."

if [ ! -d "$VENV_DIR" ]; then
  python3 -m venv "$VENV_DIR"
  echo "      Prostředí vytvořeno."
else
  echo "      Prostředí již existuje, přeskakuji."
fi

# Aktivace
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

# ── pip závislosti ─────────────────────────────────────────────────────────────
echo ""
echo "[2/3] Instaluji Python závislosti (requirements.txt) ..."

pip install --quiet --upgrade pip
pip install --quiet -r requirements.txt
echo "      Flask a další balíčky nainstalovány."

# ── Hardwarové knihovny Sequent Microsystems ──────────────────────────────────
echo ""
if [ "$INSTALL_HARDWARE" = true ]; then
  echo "[3/3] Instaluji hardwarové knihovny Sequent Microsystems ..."

  # Zkontroluj sudo
  if ! command -v sudo &>/dev/null; then
    echo "CHYBA: sudo není dostupné." >&2
    exit 1
  fi

  # Zkontroluj git
  if ! command -v git &>/dev/null; then
    echo "CHYBA: git není nainstalován. Spusť: sudo apt install git" >&2
    exit 1
  fi

  # Zkontroluj make
  if ! command -v make &>/dev/null; then
    echo "      make nenalezen, instaluji..."
    sudo apt-get install -y make
  fi

  TMPDIR=$(mktemp -d)
  trap 'rm -rf "$TMPDIR"' EXIT

  # 16-vstupní deska
  echo "      Klonovám 16inpind-rpi ..."
  git clone --quiet https://github.com/SequentMicrosystems/16inpind-rpi.git "$TMPDIR/16inpind-rpi"
  echo "      Instaluji lib16inpind (sudo make install) ..."
  sudo make -C "$TMPDIR/16inpind-rpi" install

  # 16-výstupní deska
  echo "      Klonovám 16relind-rpi ..."
  git clone --quiet https://github.com/SequentMicrosystems/16relind-rpi.git "$TMPDIR/16relind-rpi"
  echo "      Instaluji lib16relind (sudo make install) ..."
  sudo make -C "$TMPDIR/16relind-rpi" install

  # Povolit I2C
  echo ""
  echo "      Kontroluji I2C ..."
  if ! raspi-config nonint get_i2c | grep -q "0"; then
    echo "      Povoluju I2C přes raspi-config ..."
    sudo raspi-config nonint do_i2c 0
    echo "      I2C povoleno. Pro aktivaci může být potřeba restart."
  else
    echo "      I2C je již povoleno."
  fi

  echo "      Hardwarové knihovny nainstalovány."
else
  echo "[3/3] Hardwarové knihovny přeskočeny."
  echo "      Pro instalaci na Raspberry Pi spusť: bash setup.sh --hardware"
  echo "      Simulační režim (SIMULATE=True v config.py) funguje bez nich."
fi

# ── Hotovo ────────────────────────────────────────────────────────────────────
echo ""
echo "======================================"
echo "  Hotovo!"
echo ""
echo "  Aktivace prostředí:"
echo "    source $VENV_DIR/bin/activate"
echo ""
echo "  Spuštění webové aplikace:"
echo "    python webapp.py"
echo ""
echo "  Spuštění bez webového UI:"
echo "    python main.py"
echo "======================================"
