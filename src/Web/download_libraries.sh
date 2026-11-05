#!/bin/bash
#
# Downloads the javascript libraries of the web interface into lib/ (not part of the repository):
# three.js (MIT) and urdf-loader (Apache-2.0) from the npm registry.
# Only the modules used by js/*.js and the modules they import are extracted.
#
# usage: download_libraries.sh [--force]
#
set -euo pipefail

THREE_VERSION=0.187.0
THREE_SHA512=422bbb47e0a687c0acb59b18d6e24e4969d595fa3b66d34b57b985f0996d9372c1dc0b43bbd71166572744a90093c638cfa28ac37e73ed8ed4addee3598f1eb4
THREE_FILES=(
    LICENSE
    build/three.module.js
    build/three.core.js
    examples/jsm/controls/OrbitControls.js
    examples/jsm/loaders/STLLoader.js
    examples/jsm/loaders/OBJLoader.js
    examples/jsm/loaders/ColladaLoader.js
    examples/jsm/loaders/collada/ColladaParser.js
    examples/jsm/loaders/collada/ColladaComposer.js
    examples/jsm/loaders/TGALoader.js
)

URDF_LOADER_VERSION=0.13.1
URDF_LOADER_SHA512=6c71e8874631ae0af0f7574f026567cca95e03c1e254c787cbac58a8e551c372ee6768433b052241509baf33d97243b94b7d05d58ca6f332c220d3741df70cdf
URDF_LOADER_FILES=(
    src/URDFLoader.js
    src/URDFClasses.js
)
# the npm package of urdf-loader contains no license file
URDF_LOADER_LICENSE_URLS=(
    "https://raw.githubusercontent.com/gkjohnson/urdf-loaders/v$URDF_LOADER_VERSION/LICENSE"
    "https://raw.githubusercontent.com/gkjohnson/urdf-loaders/master/LICENSE"
)

WEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="$WEB_DIR/lib"
VERSIONS="three $THREE_VERSION, urdf-loader $URDF_LOADER_VERSION"

if [[ "${1:-}" != "--force" && -f "$LIB_DIR/VERSIONS" && "$(cat "$LIB_DIR/VERSIONS")" == "$VERSIONS" ]]; then
    echo "libraries are up to date in $LIB_DIR (use --force to download again)"
    exit 0
fi

download() {  # url target
    if command -v curl > /dev/null; then
        curl --fail --silent --show-error --location --output "$2" "$1"
    elif command -v wget > /dev/null; then
        wget --quiet --output-document="$2" "$1"
    else
        echo "curl or wget is required" >&2
        exit 1
    fi
}

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT
NEW_LIB="$TMP_DIR/lib"

install_package() {  # name version sha512 files...
    local name=$1 version=$2 checksum=$3
    shift 3
    local archive="$TMP_DIR/$name-$version.tgz"
    echo "downloading $name $version"
    download "https://registry.npmjs.org/$name/-/$name-$version.tgz" "$archive"
    if ! echo "$checksum  $archive" | sha512sum --check --quiet - ; then
        echo "the checksum of $name $version does not match" >&2
        exit 1
    fi
    mkdir -p "$TMP_DIR/$name"
    # npm archives contain a package/ folder
    tar -xzf "$archive" -C "$TMP_DIR/$name" "${@/#/package/}"
    for file in "$@"; do
        mkdir -p "$(dirname "$NEW_LIB/$name/$file")"
        cp "$TMP_DIR/$name/package/$file" "$NEW_LIB/$name/$file"
        echo "  lib/$name/$file"
    done
}

install_package three "$THREE_VERSION" "$THREE_SHA512" "${THREE_FILES[@]}"
install_package urdf-loader "$URDF_LOADER_VERSION" "$URDF_LOADER_SHA512" "${URDF_LOADER_FILES[@]}"

license_found=false
for url in "${URDF_LOADER_LICENSE_URLS[@]}"; do
    if download "$url" "$NEW_LIB/urdf-loader/LICENSE" 2> /dev/null; then
        license_found=true
        echo "  lib/urdf-loader/LICENSE"
        break
    fi
done
if [[ "$license_found" != true ]]; then
    rm -f "$NEW_LIB/urdf-loader/LICENSE"
    echo "warning: could not download the urdf-loader license (Apache-2.0)" >&2
fi

echo "$VERSIONS" > "$NEW_LIB/VERSIONS"
# replace the old libraries only when everything was downloaded
rm -rf "$LIB_DIR"
mv "$NEW_LIB" "$LIB_DIR"
echo "done: $LIB_DIR"
