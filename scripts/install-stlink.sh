#!/bin/bash
# This script installs st-link on a Raspberry Pi machine running the
# OctoPi distribution.

#!/bin/bash
# This script installs Klipper on a Raspberry Pi machine running the
# OctoPi distribution.

# Step 1: Install system packages
install_packages()
{
    # Packages for compiling st-link
    PKGLIST="cmake"
    # hub-ctrl
    PKGLIST="${PKGLIST} libusb-1.0-0-dev"

    # Update system package info
    report_status "Running apt-get update..."
    sudo apt-get update

    # Install desired packages
    report_status "Installing packages..."
    sudo apt-get install --yes ${PKGLIST}
}

# Step 2: Install ST-Link V2
install_stlink()
{
    report_status "Cloning ST-Link from source..."

    mkdir ~/proj
    cd ~/proj
    git clone https://github.com/texane/stlink stlink-repo
    cd stlink-repo

    report_status "Building ST-Link..."
    make

    report_status "Installing ST-Link..."
    cd build/Release
    sudo make install

    sudo rm -r ~/proj

    # make sure that libraries are linked
    sudo ldconfig
}

# Helper functions
report_status()
{
    echo -e "\n\n###### $1"
}

verify_ready()
{
    if [ "$EUID" -eq 0 ]; then
        echo "This script must not run as root"
        exit -1
    fi
}

# Force script to exit if an error occurs
set -e


# Run installation steps defined above
verify_ready
install_packages
install_stlink