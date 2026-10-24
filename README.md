# PoC: Percepção com ROS 2, C++ e OpenCV

Trabalho de Prospecção Teórica e Tecnológica da disciplina de Engenharia de Software (UFLA).

Dois nós ROS 2 em C++ processam um vídeo com OpenCV:

- `video_publisher`: lê um vídeo gravado e publica os quadros em `/camera/image_raw`.  
- `edge_detector`: assina esse tópico, converte a imagem com `cv_bridge`, aplica tons de cinza, desfoque gaussiano e Canny, e publica o resultado em `/camera/image_edges`.

## Requisitos

- Ubuntu 24.04 (no meu caso, dentro do WSL2) e ROS 2 Jazzy  
- `libopencv-dev`, `ros-jazzy-cv-bridge`, `ros-jazzy-rqt-image-view`

## Como rodar

Coloque este pacote em `~/ros2_ws/src/` e compile:

cd \~/ros2\_ws

colcon build \--packages-select poc\_percepcao

source install/setup.bash

Depois, em três terminais separados:

ros2 run poc\_percepcao video\_publisher \--ros-args \-p video\_path:=/caminho/do/video.mp4

ros2 run poc\_percepcao edge\_detector

ros2 run rqt\_image\_view rqt\_image\_view

Use qualquer vídeo `.mp4` curto. No `rqt_image_view`, escolha `/camera/image_raw` para ver o vídeo original e `/camera/image_edges` para ver os contornos detectados.