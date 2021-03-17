#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ahelmi/infant_sim_testing/src/behavior_tree"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ahelmi/infant_sim_testing/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ahelmi/infant_sim_testing/install/lib/python2.7/dist-packages:/home/ahelmi/infant_sim_testing/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ahelmi/infant_sim_testing/build" \
    "/usr/bin/python2" \
    "/home/ahelmi/infant_sim_testing/src/behavior_tree/setup.py" \
     \
    build --build-base "/home/ahelmi/infant_sim_testing/build/behavior_tree" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ahelmi/infant_sim_testing/install" --install-scripts="/home/ahelmi/infant_sim_testing/install/bin"
