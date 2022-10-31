# shellcheck shell=bash
BIN=$(readlink -f ../builddir)

# Skip Valgrind outside of Linux, and Wine outside of Linux/Mac.
# If you don't have Valgrind, add `export VALGRIND=`.
# If you don't have Wine, add `export WINE=echo` to skip running snesbrr.
unameOut="$(uname -s)"
case "${unameOut}" in
Linux*)
# TODO bsd?
    VALGRIND='valgrind --error-exitcode=128'
    WINE='wine'
    ;;
Darwin*)
    VALGRIND=
    WINE='wine64'
    ;;
*)
    VALGRIND=
    WINE=
    ;;
esac
