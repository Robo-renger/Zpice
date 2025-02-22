#!/bin/bash

# Define the paths
SETUP_FILE="setup.py"
PACKAGE_XML="package.xml"
SETUP_FILE_SRC="src/setup.py"

if [ -f "$SETUP_FILE" ]; then
  CURRENT_VERSION=$(grep -E "version\s*=\s*['\"][0-9]+\.[0-9]+\.[0-9]+['\"]" "$SETUP_FILE" | sed -E "s/.*version\s*=\s*['\"]([0-9]+\.[0-9]+\.[0-9]+)['\"].*/\1/")
  if [ -z "$CURRENT_VERSION" ]; then
    echo "Error: Could not extract version from setup.py"
    exit 1
  fi
  echo "Current version in setup.py: ${CURRENT_VERSION}"
else
  echo "Error: setup.py not found!"
  exit 1
fi

# --- Increment the patch version ---
IFS='.' read -r MAJOR MINOR PATCH <<< "$CURRENT_VERSION"
NEW_PATCH=$((PATCH + 1))
NEW_VERSION="${MAJOR}.${MINOR}.${NEW_PATCH}"
echo "New version will be: ${NEW_VERSION}"

# This sed command finds the version line and replaces the version with the new version.
sed -i.bak -E "s/(version\s*=\s*['\"])[0-9]+\.[0-9]+\.[0-9]+(['\"])/\1${NEW_VERSION}\2/" "$SETUP_FILE"
echo "Updated "$SETUP_FILE" to version ${NEW_VERSION}"

# This sed command finds the version line and replaces the version with the new version.
sed -i.bak -E "s/(version\s*=\s*['\"])[0-9]+\.[0-9]+\.[0-9]+(['\"])/\1${NEW_VERSION}\2/" "$SETUP_FILE_SRC"
echo "Updated "$SETUP_FILE_SRC" to version ${NEW_VERSION}"

if [ -f "$PACKAGE_XML" ]; then
  sed -i.bak -E "s/(<version>)[^<]+(<\/version>)/\1${NEW_VERSION}\2/" "$PACKAGE_XML"
  echo "Updated "$PACKAGE_XML" to version ${NEW_VERSION}"
else
  echo "Warning: "$PACKAGE_XML" not found! Skipping update of "$PACKAGE_XML"."
fi

pip install --no-cache-dir --force-reinstall .

cd /home/ziad/zpice_ws/src/control/src
pip install --no-cache-dir --force-reinstall .

echo "Installation complete with version ${NEW_VERSION}."