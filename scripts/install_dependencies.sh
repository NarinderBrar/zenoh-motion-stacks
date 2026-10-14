#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repository_dir=$(cd "$script_dir/.." && pwd)
assume_yes=false
dry_run=false
prefix=/usr/local

while (($#)); do
    case "$1" in
        --yes) assume_yes=true ;;
        --dry-run) dry_run=true ;;
        --prefix)
            shift
            prefix=${1:?missing prefix}
            ;;
        *)
            printf 'Unknown argument: %s\n' "$1" >&2
            exit 2
            ;;
    esac
    shift
done

source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
    printf 'Unsupported system: %s %s. Use Ubuntu 24.04 or the Dockerfile when available.\n' "$ID" "$VERSION_ID" >&2
    exit 2
fi
if [[ "$prefix" != /usr/local ]]; then
    printf 'This baseline installer uses Ubuntu packages and cannot install into %s yet.\n' "$prefix" >&2
    exit 2
fi

dependency_file="$repository_dir/cmake/dependencies.cmake"
dependency_version() {
    local value
    value=$(sed -n "s/^set(ZMS_$1_VERSION \([0-9][0-9.]*\))\$/\1/p" "$dependency_file")
    if [[ -z "$value" ]]; then
        printf 'Cannot read ZMS_%s_VERSION from %s\n' "$1" "$dependency_file" >&2
        exit 1
    fi
    printf '%s' "$value"
}
declare -A versions
versions[json]=$(dependency_version JSON)
versions[zenohc]=$(dependency_version ZENOHC)
versions[zenohcxx]=$(dependency_version ZENOHCXX)

packages=(build-essential cmake ninja-build libeigen3-dev liburdfdom-dev libompl-dev nlohmann-json3-dev libgtest-dev libzenohc-dev libzenohcpp-dev)
declare -A requirements=([nlohmann-json3-dev]="${versions[json]}" [libzenohc-dev]="${versions[zenohc]}" [libzenohcpp-dev]="${versions[zenohcxx]}")
missing=()

for package in "${packages[@]}"; do
    installed=$(dpkg-query -W -f='${Version}' "$package" 2>/dev/null || true)
    minimum=${requirements[$package]:-}
    if [[ -z "$installed" ]] || { [[ -n "$minimum" ]] && ! dpkg --compare-versions "$installed" ge "$minimum"; }; then
        missing+=("$package")
        printf '%-24s %-12s %s\n' "$package" "${installed:-missing}" "install required"
    else
        printf '%-24s %-12s %s\n' "$package" "$installed" "already installed"
    fi
done

if ((${#missing[@]} == 0)); then
    exit 0
fi

root=()
if ((EUID != 0)); then
    root=(sudo)
fi
zenoh_source=/etc/apt/sources.list.d/motion-stacks-zenoh.list
zenoh_key=/etc/apt/keyrings/zenoh-public-key.gpg
needs_zenoh_repo=false
for package in "${missing[@]}"; do
    if [[ "$package" == libzenohc-dev || "$package" == libzenohcpp-dev ]]; then
        if [[ ! -f "$zenoh_source" ]]; then
            needs_zenoh_repo=true
        fi
    fi
done

if "$dry_run"; then
    if "$needs_zenoh_repo"; then
        printf 'Would run: %sapt-get install %s ca-certificates curl gnupg\n' "${root[*]:+${root[*]} }" "$([[ "$assume_yes" == true ]] && printf '%s' '-y' || true)"
        printf 'Would add the Eclipse Zenoh signing key and apt source to %s and %s\n' "$zenoh_key" "$zenoh_source"
    fi
    printf 'Would run: %sapt-get update\n' "${root[*]:+${root[*]} }"
    printf 'Would run: %sapt-get install' "${root[*]:+${root[*]} }"
    if "$assume_yes"; then
        printf ' -y'
    fi
    printf ' %s' "${missing[@]}"
    printf '\n'
    exit 0
fi

apt_options=()
if "$assume_yes"; then
    apt_options=(-y)
fi
if "$needs_zenoh_repo"; then
    "${root[@]}" apt-get update
    "${root[@]}" apt-get install "${apt_options[@]}" ca-certificates curl gnupg
    curl --fail --location --silent --show-error https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key |
        "${root[@]}" gpg --dearmor --yes --output "$zenoh_key"
    printf 'deb [signed-by=%s] https://download.eclipse.org/zenoh/debian-repo/ /\n' "$zenoh_key" |
        "${root[@]}" tee "$zenoh_source" >/dev/null
fi

"${root[@]}" apt-get update

for package in "${missing[@]}"; do
    candidate=$(apt-cache policy "$package" | awk '/Candidate:/ {print $2}')
    if [[ -z "$candidate" || "$candidate" == '(none)' ]]; then
        printf '%s has no apt candidate. Install its development package from the upstream project first.\n' "$package" >&2
        exit 1
    fi
    minimum=${requirements[$package]:-}
    if [[ -n "$minimum" ]] && ! dpkg --compare-versions "$candidate" ge "$minimum"; then
        printf '%s candidate %s is older than required %s.\n' "$package" "$candidate" "$minimum" >&2
        exit 1
    fi
done

"${root[@]}" apt-get install "${apt_options[@]}" "${missing[@]}"
