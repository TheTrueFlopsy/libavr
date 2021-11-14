#!/bin/bash

router=$(dirname $(readlink -f $0))/inm_router.py

runuser -u $INM_ROUTER_USER -- $router $@
