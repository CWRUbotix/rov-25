# TODO (REST OF README)

## Calibration

From [this guide](https://docs.luxonis.com/hardware/platform/depth/calibration).

Install depthai repo (installed to `~/depthai` on competition laptop):

```bash
git clone https://github.com/luxonis/depthai.git
cd depthai
git submodule update --init --recursive

python3 -m venv venv
. venv/bin/activate

python3 install_requirements.py
```

Run the calibration script (`-s` is square size in cm, `-nx` & `-ny` are grid dimensions in # squares) to take pictures:

```bash
python3 ~/depthai/calibrate.py -s 2.208 -nx 12 -ny 9 -brd ~/rov-25/src/surface/luxonis_cam/calibration/rov_depth_enclosure.json
```

> Optionally get on the `develop` branch if you get errors with our cams (`OV9782` RGB, not `OV9282` mono) being detected incorrectly:
>
> ```
> git checkout develop
> ```

Images will be saved to `depthai/dataset`. Press `s` to continue to calibration. Calibration results are stored in `depthai/resources`.

Optionally, rerun calibration in the images in `dataset`:

```bash
python3 ~/depthai/calibrate.py -s 2.208 -nx 12 -ny 9 -brd ~/rov-25/src/surface/luxonis_cam/calibration/rov_depth_enclosure.json -m process
```