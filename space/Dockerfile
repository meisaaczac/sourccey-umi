FROM python:3.11-slim

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 ffmpeg && \
    rm -rf /var/lib/apt/lists/*

COPY space/requirements.txt /app/space/requirements.txt
RUN pip install --no-cache-dir -r /app/space/requirements.txt

COPY software/ /app/software/
COPY tools/ /app/tools/
COPY space/ /app/space/

ENV PORT=7860
EXPOSE 7860

CMD ["python", "space/app.py"]
