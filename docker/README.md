# Docker Files

## Build Docker

Run this command from within the `docker` directory to build the image.

```bash
docker build . -t motion3d:latest -f motion3d.dockerfile
```

## Run Docker

Execute the following command from the project root to run the docker container, mount the current directory and use it as workspace.

```bash
docker run -it -v $(pwd):$(pwd):rw -w $(pwd) --rm motion3d:latest /bin/bash
```
