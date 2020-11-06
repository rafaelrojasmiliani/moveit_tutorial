
@ECHO OFF

docker build -t "moveit_docker" ^
    --build-arg myuser="%USERNAME%" ^
    --build-arg myuid=11011 ^
    --build-arg mygroup="%USERNAME%" ^
    --build-arg mygid=11011 ^
      -f ./image.dockerfile .
