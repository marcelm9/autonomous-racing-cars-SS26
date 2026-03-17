#!/usr/bin/env bash

old_envs=$(env | sort)

# shellcheck disable=SC1090
source "$1"

new_envs=$(env | sort)

diff_envs=$(comm -13 <(echo "$old_envs") <(echo "$new_envs"))

diff_oneline=$(echo "$diff_envs" | tr '\n' ';')

echo "$diff_oneline"