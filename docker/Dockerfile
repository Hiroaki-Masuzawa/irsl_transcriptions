# ベースイメージを引数で指定可能（例: python:3.10-slim）
ARG BASE_IMAGE=python:3.10-slim
FROM ${BASE_IMAGE}

ENV IRSL_WS=/irsl_catkin_ws
ENV IRSL_PROJECT=irsl_transcriptions

WORKDIR ${IRSL_WS}

# 環境変数でPythonの出力をバッファリングせず即時表示
ENV PYTHONUNBUFFERED=1

# 必要なシステムパッケージをインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg \
    libmagic1 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-pip python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# pytorch install 
RUN pip install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 --index-url https://download.pytorch.org/whl/cu121 && \
    pip cache purge

# Pythonパッケージのインストール
# whisperはGitHubからインストール、その他依存も
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir \
        git+https://github.com/openai/whisper.git \
        ffmpeg-python \
        python-magic
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir numpy==1.24.4
    
RUN mkdir -p ${IRSL_WS}/src
COPY . ${IRSL_WS}/src/${IRSL_PROJECT}/.
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin config --install && IRSL_BUILD_LOCAL=true catkin build"


CMD [ "bash", "-c", "source ${IRSL_WS}/install/setup.bash && roslaunch ${IRSL_PROJECT} run.launch" ]
