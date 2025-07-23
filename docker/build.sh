SCRIPT_DIR=$(cd $(dirname $0); pwd)
docker build --build-arg BASE_IMAGE=repo.irsl.eiiris.tut.ac.jp/irsl_base:noetic_opengl -t irsl_transcriptions -f ${SCRIPT_DIR}/Dockerfile ${SCRIPT_DIR}/..
