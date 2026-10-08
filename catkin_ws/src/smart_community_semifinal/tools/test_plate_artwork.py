#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Plate artwork/metric contracts; projected frames below are SYNTHETIC, not Gazebo."""
from __future__ import division, print_function, unicode_literals
import hashlib
import io
import json
import os
import random
import sys
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG, 'scripts'))
from reference_detector import ReferenceDetector, imread
from plate_ocr import PlateCharacterRecognizer
from image_geometry import planar_position
from make_random_plates import (EXAMPLES, DEFAULT_SEED, character_bank,
                                compose, random_picks, slot_pixels)


class PlateArtworkContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest_path = os.path.join(PKG, 'assets', 'manifest.json')
        with io.open(cls.manifest_path, encoding='utf-8') as stream:
            manifest = json.load(stream)
        cls.plates = [row for row in manifest['recognition_assets'] if row['category'] == 'plate']
        cls.examples = {}
        for index, (key, unused, label) in enumerate(EXAMPLES, 1):
            image = imread(os.path.join(PKG, 'tools', 'fixtures', 'official_plates',
                                       'plate_example_%d.png' % index))
            image = cv2.resize(image, (PlateCharacterRecognizer.WIDTH,
                                      PlateCharacterRecognizer.HEIGHT))
            cls.examples[key] = (cv2.cvtColor(image, cv2.COLOR_BGR2RGB), label)
        cls.ocr = PlateCharacterRecognizer([(image[:, :, ::-1], label)
                                            for image, label in cls.examples.values()])
        cls.detector = ReferenceDetector(cls.manifest_path)

    def test_plate_reference_texture_and_metric_size_agree(self):
        self.assertEqual(len(self.plates), 3)
        for row in self.plates:
            reference_path = os.path.join(PKG, 'assets', row['file'])
            texture_path = os.path.join(PKG, 'models', os.path.splitext(row['file'])[0],
                                        'materials', 'textures', 'texture.png')
            reference, texture = imread(reference_path), imread(texture_path)
            self.assertEqual(reference.shape[:2], (120, 380), row['file'])
            self.assertTrue(np.array_equal(reference, texture), row['file'])
            self.assertEqual(reference.shape[1] * 6, reference.shape[0] * 19)
            for width, height in (('width_m', 'height_m'),
                                  ('artwork_width_m', 'artwork_height_m')):
                self.assertAlmostEqual(row[width], .095)
                self.assertAlmostEqual(row[height], .03)
            with open(reference_path, 'rb') as stream:
                self.assertEqual(hashlib.sha256(stream.read()).hexdigest(), row['reference_sha256'])

    def test_plate_has_no_added_white_or_black_vertical_strips(self):
        for row in self.plates:
            image = imread(os.path.join(PKG, 'assets', row['file']))
            white_columns = np.all(image >= 245, axis=(0, 2))
            black_columns = np.all(image <= 5, axis=(0, 2))
            self.assertFalse(white_columns.any(), row['file'])
            self.assertFalse(black_columns.any(), row['file'])

    def test_plate_model_geometry_keeps_official_dimensions(self):
        for index in range(1, 4):
            model = os.path.join(PKG, 'models', 'plate_%d' % index)
            root = ET.parse(os.path.join(model, 'model.sdf')).getroot()
            size = root.find('.//collision/geometry/box/size').text.split()
            self.assertAlmostEqual(float(size[0]), .095)
            self.assertAlmostEqual(float(size[2]), .03)
            mesh = ET.parse(os.path.join(model, 'meshes', 'card.dae')).getroot()
            ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
            positions = mesh.find('.//c:source/c:float_array', ns)
            vertices = np.array([float(x) for x in positions.text.split()]).reshape(-1, 3)
            self.assertAlmostEqual(float(np.ptp(vertices[:, 0])), .095)
            self.assertAlmostEqual(float(np.ptp(vertices[:, 2])), .03)

    def test_composition_preserves_background_outside_character_slots(self):
        bank = character_bank(self.examples)
        rng = random.Random(DEFAULT_SEED)
        untouched = np.ones(PlateCharacterRecognizer.WIDTH, dtype=bool)
        for x0, x1 in slot_pixels():
            untouched[x0:x1] = False
        for unused in range(20):
            picks = random_picks(bank, rng)
            image = compose(self.examples, picks)
            background = self.examples[picks[0][1]][0]
            self.assertTrue(np.array_equal(image[:, untouched], background[:, untouched]))
            self.assertFalse(np.all(image == 0, axis=(0, 2)).any())
            for index, (x0, x1) in enumerate(slot_pixels()):
                self.assertTrue(np.array_equal(image[:, x0:x1],
                                               self.examples[picks[index][1]][0][:, x0:x1]))

    def test_composition_does_not_mutate_official_examples(self):
        originals = {key: image.copy() for key, (image, unused) in self.examples.items()}
        compose(self.examples, random_picks(character_bank(self.examples), random.Random(DEFAULT_SEED)))
        for key, (image, unused) in self.examples.items():
            self.assertTrue(np.array_equal(originals[key], image))

    def test_default_seed_keeps_both_random_numbers_and_ocr(self):
        rng = random.Random(DEFAULT_SEED)
        bank = character_bank(self.examples)
        for expected in ('苏DB812A', '鄂DP8522'):
            picks = random_picks(bank, rng)
            self.assertEqual(''.join(char for char, unused in picks), expected)
            image = compose(self.examples, picks)
            result = self.ocr.recognize(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
            self.assertTrue(result['complete'], result)
            self.assertEqual(result['text'], expected)

    def test_source_hashes_and_route_labels_remain_consistent(self):
        with io.open(os.path.join(PKG, 'config', 'layout.json'), encoding='utf-8') as stream:
            layout = json.load(stream)
        labels = {point['expected_label'] for point in layout['route']
                  if point.get('expected_category') == 'plate'}
        self.assertEqual(labels, {row['label'] for row in self.plates})
        for row in self.plates:
            if row['text_status'] == 'randomly_generated':
                with open(os.path.join(PKG, row['source']), 'rb') as stream:
                    self.assertEqual(hashlib.sha256(stream.read()).hexdigest(), row['sha256'])

    def test_person_letterbox_and_artwork_dimensions_are_not_plate_dimensions(self):
        with io.open(self.manifest_path, encoding='utf-8') as stream:
            manifest = json.load(stream)
        for row in manifest['recognition_assets']:
            if row['category'] not in ('resident', 'visitor'):
                continue
            texture = imread(os.path.join(PKG, 'models', os.path.splitext(row['file'])[0],
                                         'materials', 'textures', 'texture.png'))
            self.assertAlmostEqual(row['width_m'], .05)
            self.assertAlmostEqual(row['height_m'], .15)
            self.assertLessEqual(abs(texture.shape[1] / texture.shape[0] - 1 / 3), .003)
            self.assertLessEqual(row['artwork_width_m'], row['width_m'])
            self.assertLessEqual(row['artwork_height_m'], row['height_m'])

    def test_synthetic_official_size_plates_match_and_recover_metric_pose(self):
        # No Gazebo claim: pin the reference -> homography -> OCR -> PnP chain
        # on physically projected current textures at three poses for each plate.
        camera = np.float64([[1100., 0., 640.], [0., 1100., 480.], [0., 0., 1.]])
        objects = np.float64([[-.095/2, -.03/2, 0.], [.095/2, -.03/2, 0.],
                              [.095/2, .03/2, 0.], [-.095/2, .03/2, 0.]])
        poses = ((.46, 0., 0.), (.55, .18, -.04), (.62, -.25, .04))
        for row in self.plates:
            image = imread(os.path.join(PKG, 'assets', row['file']))
            height, width = image.shape[:2]
            source = np.float32([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]])
            for distance, yaw, roll in poses:
                target_xyz = np.float64([.01, .05, distance])
                rvec = np.float64([0., yaw, roll])
                corners, unused = cv2.projectPoints(objects, rvec, target_xyz,
                                                   camera, np.zeros(5))
                matrix = cv2.getPerspectiveTransform(source, corners.reshape(4, 2).astype(np.float32))
                frame = cv2.warpPerspective(image, matrix, (1280, 960),
                                            borderValue=(180, 180, 180))
                cv2.setRNGSeed(17)
                detections = self.detector.detect(frame, categories=('plate',))
                self.assertEqual([d['label'] for d in detections], [row['label']],
                                 '%s distance=%.2f yaw=%.2f: %r' % (row['file'], distance, yaw, detections))
                detection = detections[0]
                # OCR is independent of reference identification. At oblique
                # views it may reject a weak character, never substitute a
                # wrong one. Require complete character output head-on.
                ocr = detection['ocr_result']
                self.assertEqual(len(ocr['characters']), len(row['label']))
                for index, character in enumerate(ocr['characters']):
                    if character['accepted']:
                        self.assertEqual(character['value'], row['label'][index], ocr)
                    else:
                        self.assertEqual(character['value'], '?', ocr)
                if yaw == 0. and roll == 0.:
                    self.assertTrue(detection['character_ocr'], ocr)
                    self.assertEqual(ocr['text'], row['label'])
                position = planar_position(detection, camera)
                self.assertIsNotNone(position, row['file'])
                error = np.linalg.norm(np.array(position['camera_xyz']) - target_xyz)
                self.assertLess(error, .02, '%s metric position error %.4f m' % (row['file'], error))


if __name__ == '__main__':
    unittest.main()
