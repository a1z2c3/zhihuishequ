#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Pin reference-matching quality on real Gazebo frames.

Why this file exists
--------------------
A change that claimed to "make the confidence genuinely higher" (CLAHE plus a
finer ORB pyramid) actually cut total RANSAC inliers by 27.7% on real VM
frames and turned a 75-inlier detection into zero, while every other offline
test stayed green: nothing in the suite looked at matching at all.  These
contracts close that hole.

The two fixtures are evidence frames captured by a real VM run
(patrol_20260927_154517_6CTdOK).  They are annotated by the detector that
produced them, so matching on them is slightly perturbed -- they are the only
real-camera data in the repository and they exercise the exact production code
path.  Thresholds are deliberately loose: this guards against a collapse
(a label disappearing, the inlier count falling to the gate) without pinning
OpenCV-version-specific match counts, because the same suite also runs on the
teaching VM's older OpenCV.
"""
from __future__ import division, print_function, unicode_literals
import io
import os
import sys
import unittest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG, 'scripts'))

from reference_detector import (ReferenceDetector, imread,
                                MATCH_MAX_DISTANCE, MATCH_MAX_RATIO)
from semifinal_core import temporal_confidence

FIXTURES = os.path.join(PKG, 'tools', 'fixtures', 'real_frames')
MANIFEST = os.path.join(PKG, 'assets', 'manifest.json')

# Ground truth from the run's own event log for these two frames.
PERSON_FRAME_TRUTH = {'resident_9', 'resident_12', 'resident_6',
                      'resident_11', 'resident_2'}
PLATE_FRAME_TRUTH = {u'苏AB8Q62'}
PERSON_CATEGORIES = ('resident', 'visitor')


def load(name):
    return imread(os.path.join(FIXTURES, name))


class ReferenceMatchingContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector = ReferenceDetector(MANIFEST)

    def test_real_person_frame_still_matches(self):
        """A real person view must keep producing inliers, not silence.

        The rejected CLAHE configuration reported no reference at all here.
        Measured on this fixture with the shipped configuration: one reference
        with 11-12 inliers at a score of 0.72-0.76.  Raising nfeatures to 3000
        lifts that to three references and 69 inliers, but it costs 58% more
        time per frame and the observation handshake only has five frames per
        2.0 s, so the thresholds below pin the shipped configuration rather
        than the better one we cannot yet afford.
        """
        image = load('person_view_a.png')
        self.assertIsNotNone(image, 'person fixture missing')
        got = self.detector.detect(image, categories=PERSON_CATEGORIES)
        labels = set(d['label'] for d in got)
        self.assertTrue(labels, 'no reference matched on a real person frame')
        self.assertTrue(labels <= PERSON_FRAME_TRUTH,
                        'spurious labels on a real person frame: %s' %
                        sorted(labels - PERSON_FRAME_TRUTH))
        total = sum(d['inliers'] for d in got)
        self.assertGreaterEqual(total, 10,
                                'inlier budget collapsed to %d' % total)
        for d in got:
            self.assertGreaterEqual(d['inliers'], 10)
            self.assertGreater(d['confidence'], 0.60)

    def test_real_plate_frame_still_matches(self):
        image = load('plate_view_a.png')
        self.assertIsNotNone(image, 'plate fixture missing')
        got = self.detector.detect(image, categories=('plate',))
        labels = set(d['label'] for d in got)
        self.assertTrue(labels <= PLATE_FRAME_TRUTH,
                        'spurious plate labels: %s' % sorted(labels))
        self.assertIn(u'苏AB8Q62', labels, 'the real plate was not matched')
        plate = [d for d in got if d['label'] == u'苏AB8Q62'][0]
        self.assertGreaterEqual(plate['inliers'], 40,
                                'plate inliers collapsed to %d' % plate['inliers'])
        self.assertGreater(plate['confidence'], 0.80)

    def test_detections_expose_the_evidence_they_were_scored_from(self):
        """The published score must stay auditable, factor by factor."""
        got = self.detector.detect(load('plate_view_a.png'), categories=('plate',))
        self.assertTrue(got)
        d = got[0]
        for key in ('raw_matching_score', 'score_factors', 'inliers',
                    'photometric_correlation'):
            self.assertIn(key, d)
        for key in ('ratio', 'inliers', 'spread', 'photometric'):
            self.assertIn(key, d['score_factors'])
            self.assertGreaterEqual(d['score_factors'][key], 0.0)
            self.assertLessEqual(d['score_factors'][key], 1.0)


class ConfidenceScoringContracts(unittest.TestCase):
    def test_no_hidden_floor_in_the_published_score(self):
        """A detection that barely cleared the gates must NOT be rescued.

        Every acceptance gate is satisfied exactly at its boundary here:
        ratio 0.40, 10 inliers, a quarter of the required spread, and a
        photometric correlation sitting exactly on the bound.  The published
        score has to show that, so the old "0.50 baseline + 0.50 x evidence"
        remap -- which would have printed 0.67 for this match -- must be gone.
        """
        score = ReferenceDetector._confidence_from_evidence(
            'resident', 0.05, 0.40, 10 / 16.0, 0.25, 0.50, 0.50)
        expected = 0.35 * 0.40 + 0.25 * (10 / 16.0) + 0.20 * 0.25 + 0.20 * 0.0
        self.assertAlmostEqual(score, expected, places=4)
        self.assertLess(score, 0.50,
                        'a boundary-quality match was given a comfortable score')

    def test_stronger_evidence_never_scores_lower(self):
        base = ReferenceDetector._confidence_from_evidence(
            'resident', 0.20, 0.55, 0.60, 0.40, 0.70, 0.50)
        for ratio, inlier, spread, corr in ((0.70, 0.60, 0.40, 0.70),
                                            (0.55, 0.90, 0.40, 0.70),
                                            (0.55, 0.60, 0.90, 0.70),
                                            (0.55, 0.60, 0.40, 0.95)):
            self.assertGreaterEqual(
                ReferenceDetector._confidence_from_evidence(
                    'resident', 0.20, ratio, inlier, spread, corr, 0.50), base)

    def test_plates_keep_their_raw_product_as_a_floor(self):
        """A plate's stricter raw score must not be lowered by the fusion."""
        score = ReferenceDetector._confidence_from_evidence(
            'plate', 0.98, 0.60, 0.60, 0.60, 0.90, 0.76)
        self.assertGreaterEqual(score, 0.98)

    def test_evidence_kind_is_not_claimed_to_be_calibrated(self):
        """The evidence file must not advertise a calibration that is gone."""
        line = EvidenceKindProbe.record_line()
        self.assertNotIn('calibrated', line)
        self.assertIn('evidence_fused', line)


class EvidenceKindProbe(object):
    """Write one evidence line into a temporary directory and hand it back."""

    @staticmethod
    def record_line():
        import json
        import shutil
        import tempfile
        from semifinal_core import EvidenceWriter
        directory = tempfile.mkdtemp(prefix='semifinal_evidence_kind_')
        try:
            writer = EvidenceWriter(directory, run_id='probe',
                                    image_writer=lambda path, image: True)
            detection = {'category': 'resident', 'label': 'resident_1',
                         'confidence': 0.9, 'bbox': [0, 0, 10, 10]}
            import numpy as np
            line = writer.record(1, 1.0, [detection], np.zeros((4, 4, 3),
                                                              dtype=np.uint8))
            self_check = json.loads(line)
            assert self_check['confidence_kind']
            return line
        finally:
            shutil.rmtree(directory, ignore_errors=True)


class TemporalConfidenceContracts(unittest.TestCase):
    def test_history_is_discarded_after_a_long_gap(self):
        """A reappearing instance must not inherit a stale high score.

        Without this, an object unseen for many frames would be vouched for by
        an old good match the moment a fresh -- and possibly wrong -- match
        arrives, which is exactly the kind of hidden inflation the score must
        not do.
        """
        history = [0.95, 0.96, 0.94]
        fresh = 0.40
        _, stale_fused, _ = temporal_confidence(history, fresh, frames_since=1)
        _, reset_fused, retained = temporal_confidence(history, fresh,
                                                      frames_since=99)
        self.assertLess(reset_fused, stale_fused)
        self.assertAlmostEqual(reset_fused, fresh, places=6)
        self.assertEqual(len(retained), 1)

    def test_one_blurred_frame_cannot_dominate(self):
        median, fused, retained = temporal_confidence([.78, .82, .80], .31)
        self.assertAlmostEqual(median, .79, places=6)
        self.assertGreater(fused, .31)
        self.assertEqual(len(retained), 4)

    def test_invalid_input_is_rejected_not_silently_clamped(self):
        for bad in (float('nan'), -0.1, 1.1, 'x'):
            with self.assertRaises(ValueError):
                temporal_confidence([], bad)


class MatchingParameterContracts(unittest.TestCase):
    def test_match_caps_are_shared_constants(self):
        """The caps must be the named constants, not scattered magic numbers."""
        detector = ReferenceDetector(MANIFEST)
        self.assertEqual(detector.match_distance, MATCH_MAX_DISTANCE)
        self.assertEqual(detector.match_ratio, MATCH_MAX_RATIO)

    def test_reference_table_keeps_every_artwork(self):
        """Every resident/visitor/plate artwork must carry descriptors."""
        detector = ReferenceDetector(MANIFEST)
        wanted = [i for i in detector.manifest['recognition_assets']
                  if i['category'] in ('resident', 'visitor', 'plate')]
        self.assertEqual(len(detector.references), len(wanted))
        for item, reference, keypoints, desc in detector.references:
            self.assertIsNotNone(desc, item['label'])
            self.assertGreater(len(keypoints), 0, item['label'])


if __name__ == '__main__':
    unittest.main()
