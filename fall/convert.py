from rknn.api import RKNN

ONNX_MODEL = 'fall.onnx'
RKNN_MODEL = 'fall.rknn'

rknn = RKNN()

print('--> Config')

rknn.config(
    mean_values=[[0,0,0]],
    std_values=[[255,255,255]],
    target_platform='rk3588'
)

print('--> Load ONNX')

ret = rknn.load_onnx(
    model=ONNX_MODEL
)

if ret != 0:
    print('load failed')
    exit(ret)

print('--> Build')

ret = rknn.build(
    do_quantization=False
)

if ret != 0:
    print('build failed')
    exit(ret)

print('--> Export')

rknn.export_rknn(RKNN_MODEL)

rknn.release()
