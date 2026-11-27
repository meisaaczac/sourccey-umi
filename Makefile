.PHONY: setup parts cad images demo space validate test all
setup:    ; pip install -r requirements.txt
parts:    ; python tools/prepare_sourccey_parts.py      # download and prepare the REAL Sourccey base and finger
cad:      ; python cad/umi_cad.py                        # generate lever and grip (STL and STEP)
images:   ; python docs/make_images.py                   # step-by-step CAD diagrams
demo:     ; python docs/render_demo_video.py             # render UMI -> Sourccey robot demo video & GIF
space:    ; python space/app.py                          # launch Sourccey UMI Studio web app (:7860)
validate: ; python validation/run_all.py                 # all validations; writes validation/RESULTS.md
test:     ; python software/test_software.py && python software/test_pipeline.py && python software/test_grabette_lerobot.py && python software/test_mono_slam.py && python software/test_v2_features.py
all: setup parts cad validate images
