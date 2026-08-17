# Model asset

`best.onnx` is the YOLO pose model used by the hybrid initialization and relock path.

- SHA-256: `2EB7BC53384650EF3BE242AD6BB0F768E60B543F31C2D77ED94D9BD6B02BC7DC`
- Expected output shape: `[1, 300, 16]`
- Runtime: ONNX Runtime when available, otherwise OpenCV DNN

The repository does not yet contain complete training-data provenance or third-party redistribution metadata for this weight file. Verify the model and dataset terms before redistributing the weight independently. The repository's MIT license does not automatically replace third-party model or dataset terms.
