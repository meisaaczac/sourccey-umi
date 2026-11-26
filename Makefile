.PHONY: setup parts cad images validate test all
setup:    ; pip install -r requirements.txt
parts:    ; python tools/prepare_sourccey_parts.py      # download and prepare the REAL Sourccey base and finger
cad:      ; python cad/umi_cad.py                        # generate lever and grip (STL and STEP)
images:   ; python docs/make_images.py                   # step-by-step images
validate: ; python validation/run_all.py                 # all validations; writes validation/RESULTS.md
test:     ; python software/test_software.py && python software/test_pipeline.py
all: setup parts cad validate images
