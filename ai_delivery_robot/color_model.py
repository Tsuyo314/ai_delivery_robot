"""Training and feature extraction for the package color classifier."""

import math

import cv2
import numpy as np


CLASS_NAMES = ('red', 'green', 'blue')


def hsv_feature(hue, saturation, value):
    """Convert OpenCV HSV values to a feature vector with circular hue."""
    angle = float(hue) * math.pi / 90.0
    return np.array([
        math.cos(angle),
        math.sin(angle),
        float(saturation) / 255.0,
        float(value) / 255.0,
    ], dtype=np.float32)


def train_color_classifier(seed=7, samples_per_class=600):
    """Train a deterministic classifier on simulated lighting variations."""
    rng = np.random.default_rng(seed)
    hue_centers = (0.0, 60.0, 115.0)
    features = []
    labels = []

    for label, center in enumerate(hue_centers):
        hues = np.mod(rng.normal(center, 7.0, samples_per_class), 180.0)
        saturations = rng.uniform(90.0, 255.0, samples_per_class)
        values = rng.uniform(65.0, 255.0, samples_per_class)
        for hue, saturation, value in zip(hues, saturations, values):
            features.append(hsv_feature(hue, saturation, value))
            labels.append(label)

    model = cv2.ml.RTrees_create()
    model.setMaxDepth(12)
    model.setMinSampleCount(2)
    model.setMaxCategories(len(CLASS_NAMES))
    model.setCalculateVarImportance(True)
    model.setTermCriteria((cv2.TERM_CRITERIA_MAX_ITER, 120, 0.0))
    trained = model.train(
        np.asarray(features, dtype=np.float32),
        cv2.ml.ROW_SAMPLE,
        np.asarray(labels, dtype=np.int32),
    )
    if not trained:
        raise RuntimeError('failed to train the package color classifier')
    return model


def predict_color(model, feature):
    """Return the predicted class name and tree-vote confidence."""
    sample = np.asarray(feature, dtype=np.float32).reshape(1, -1)
    _, predictions = model.predict(sample)
    label = int(predictions[0, 0])

    votes = model.getVotes(sample, 0)
    class_labels = votes[0]
    class_votes = votes[1]
    matching = np.flatnonzero(class_labels == label)
    confidence = 0.0
    if matching.size and class_votes.sum() > 0:
        confidence = float(class_votes[matching[0]] / class_votes.sum())
    return CLASS_NAMES[label], confidence
