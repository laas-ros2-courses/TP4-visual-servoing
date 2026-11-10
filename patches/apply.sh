#!/usr/bin/env bash
# Applique les patchs sur les paquets externes fraichement recuperes via
# `vcs import src < tiago.repos`. A lancer une seule fois juste apres
# l'import (idempotent : relancer sur un arbre deja patche echoue
# proprement sans rien casser, cf. 'patch --forward').
set -euo pipefail
cd "$(dirname "$0")/.."

for p in patches/*.patch; do
  echo "Applying $p"
  patch -p1 --forward -r /dev/null < "$p"
done
