#!/usr/bin/env bash
set -e

COMMAND="${1:-"-l"}"

rs-fw-update $COMMAND

read -p "Do you want to update a single connected D435 camera ? (Y/n) " answer
case ${answer} in
n|N)
    ;;
*)
    rs-fw-update -f Signed_Image_UVC_5_12_7_100.bin
    ;;
esac