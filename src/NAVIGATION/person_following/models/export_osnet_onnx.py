import torch
import torchreid

def export_model():
    model = torchreid.models.build_model(
        name='osnet_x0_5',
        num_classes=1000,
        pretrained=True
    )

    model.classifier = torch.nn.Identity()
    model.eval()

    dummy = torch.randn(1, 3, 256, 128)

    torch.onnx.export(
        model,
        dummy,
        "osnet_x0_5.onnx",
        input_names=["input"],
        output_names=["embedding"],
        opset_version=11,
        dynamic_axes={"input": {0: "batch"}, "embedding": {0: "batch"}},
    )

    print("Export complete: osnet_x0_5.onnx")

if __name__ == "__main__":
    export_model()
