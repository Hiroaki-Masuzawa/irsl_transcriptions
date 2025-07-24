#!/bin/bash
abs_script=$(readlink -f "$0")
abs_dir=$(dirname ${abs_script})

docker build --build-arg BASE_IMAGE=repo.irsl.eiiris.tut.ac.jp/irsl_base:cuda_12.1.0-cudnn8-devel-ubuntu22.04_one -t irsl_transcriptions -f ${abs_dir}/Dockerfile ${abs_dir}/..
