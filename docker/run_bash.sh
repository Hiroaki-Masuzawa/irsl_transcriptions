#!/bin/bash
abs_script=$(readlink -f "$0")
abs_dir=$(dirname ${abs_script})

set -x 

docker run -it --rm \
${rosip_option} ${roshostname_option} ${rosmaster_option} \
--net host \
-v ${abs_dir}/..:/userdir \
-w /userdir \
--name irsl_transcriptions \
irsl_transcriptions \
bash
