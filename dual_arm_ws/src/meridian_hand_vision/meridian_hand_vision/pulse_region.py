"""Estimate and stabilize an approximate radial-pulse region from RGB-D data."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace
import math

import numpy as np

from .branch_local_recovery_shadow import analyze_branch_local_recovery_shadow
from .branch_parent_topology_shadow import (
    analyze_branch_parent_topology_shadow,
)
from .parent_component_split_shadow import (
    analyze_parent_component_split_shadow,
)


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for the registered color image."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_camera_info(cls, message) -> "CameraIntrinsics | None":
        if (
            len(message.k) < 6
            or message.width <= 0
            or message.height <= 0
            or message.k[0] <= 0.0
            or message.k[4] <= 0.0
        ):
            return None
        return cls(
            fx=float(message.k[0]),
            fy=float(message.k[4]),
            cx=float(message.k[2]),
            cy=float(message.k[5]),
            width=int(message.width),
            height=int(message.height),
        )


@dataclass(frozen=True)
class PulseRegion:
    """Approximate radial-pulse region in image and camera coordinates."""

    pixel: tuple[int, int]
    point_m: tuple[float, float, float]
    radius_m: float


@dataclass(frozen=True)
class ArmDirection:
    """Approximate palm-to-forearm axis in the optical camera frame."""

    vector: tuple[float, float, float]


@dataclass(frozen=True)
class ForearmSection:
    """Robust 3D center of one image-space forearm cross-section."""

    section_index: int
    center_m: tuple[float, float, float] | None
    longitudinal_px: float
    point_count: int
    radius_m: float | None
    depth_spread_m: float | None
    valid: bool
    inlier: bool = False


@dataclass(frozen=True)
class ForearmAxisFit:
    """Diagnostic result of the primary mask/depth forearm-axis fit."""

    direction: ArmDirection | None
    origin_m: tuple[float, float, float] | None
    axis_length_m: float | None
    rms_m: float | None
    confidence: float
    section_count: int
    inlier_section_count: int
    inlier_ratio: float
    pca_ratio: float | None
    valid: bool
    reason: str
    quality_level: str = "PRIMARY_REJECTED"
    valid_section_count: int = 0
    max_consecutive_inlier_sections: int = 0
    max_consecutive_inlier_span_m: float = 0.0
    sections: tuple[ForearmSection, ...] = ()


@dataclass(frozen=True)
class WristGeometrySection:
    """Physical 3D section of one wrist-connected branch."""

    section_index: int
    axial_position_m: float
    center_m: tuple[float, float, float] | None
    point_count: int
    depth_spread_m: float | None
    valid: bool
    inlier: bool = False


@dataclass(frozen=True)
class WristGeometryLocalGeometry:
    """Local shape diagnostics for one wrist-connected branch."""

    linearity: float | None
    planarity: float | None
    scattering: float | None
    local_axis_p95_deg: float | None

    def diagnostics(self) -> dict:
        return {
            "linearity": self.linearity,
            "planarity": self.planarity,
            "scattering": self.scattering,
            "local_axis_p95_deg": self.local_axis_p95_deg,
        }


@dataclass(frozen=True)
class WristGeometryBranchDescriptor:
    """Compact wrist-relative descriptor used only for temporal matching."""

    axis: tuple[float, float, float] | None
    centroid_m: tuple[float, float, float]
    near_endpoint_m: tuple[float, float, float]
    far_endpoint_m: tuple[float, float, float]
    effective_length_m: float
    median_width_m: float | None

    def diagnostics(self) -> dict:
        return {
            "axis": self.axis,
            "centroid_m": self.centroid_m,
            "near_endpoint_m": self.near_endpoint_m,
            "far_endpoint_m": self.far_endpoint_m,
            "effective_length_m": self.effective_length_m,
            "median_width_m": self.median_width_m,
        }


@dataclass(frozen=True)
class WristGeometryRadiusShape:
    """Section-wise transverse radius evidence for a branch."""

    profile_m: tuple[float | None, ...] = (None,) * 8
    gradient: float | None = None
    smoothness: float | None = None
    jump: float | None = None
    valid_ratio: float = 0.0

    def diagnostics(self) -> dict:
        return {
            "radius_profile": self.profile_m,
            "radius_gradient": self.gradient,
            "radius_smoothness": self.smoothness,
            "radius_jump": self.jump,
            "radius_valid_ratio": self.valid_ratio,
        }


@dataclass(frozen=True)
class WristGeometryTopologyFeatures:
    """Parent-cloud topology observed around one directional branch."""

    distal_branch_count: int | None = None
    distal_angular_spread_deg: float | None = None
    far_shell_component_count: int | None = None
    distal_valid: bool = False
    wrist_bottleneck_radius_m: float | None = None
    growth_60mm: float | None = None
    max_local_growth: float | None = None
    growth_concentration: float | None = None
    bottleneck_ratio: float | None = None
    bottleneck_valid: bool = False

    def diagnostics(self) -> dict:
        return {
            "distal_branch_count": self.distal_branch_count,
            "distal_angular_spread": self.distal_angular_spread_deg,
            "far_shell_component_count": self.far_shell_component_count,
            "distal_valid": self.distal_valid,
            "wrist_bottleneck_radius": self.wrist_bottleneck_radius_m,
            "growth_60mm": self.growth_60mm,
            "max_local_growth": self.max_local_growth,
            "growth_concentration": self.growth_concentration,
            "bottleneck_ratio": self.bottleneck_ratio,
            "bottleneck_valid": self.bottleneck_valid,
        }


@dataclass(frozen=True)
class WristGeometryDepthSurfaceFeatures:
    """Shadow-only local surface geometry for one wrist branch."""

    local_axis: tuple[float, float, float] | None = None
    local_lateral: tuple[float, float, float] | None = None
    local_normal: tuple[float, float, float] | None = None
    transverse_curvature: float | None = None
    axial_curvature: float | None = None
    curvature_anisotropy: float | None = None
    radius_estimate_m: float | None = None
    curvature_score: float = 0.5
    curvature_valid: bool = False
    normal_axial_mad_deg: float | None = None
    normal_jump_deg: float | None = None
    normal_valid_ratio: float = 0.0
    normal_score: float = 0.5
    normal_valid: bool = False
    object_boundary_ratio: float | None = None
    internal_cut_boundary_ratio: float | None = None
    boundary_sample_count: int = 0
    boundary_score: float = 0.5
    boundary_valid: bool = False
    score_raw: float = 0.5
    evidence: float = 0.0

    def diagnostics(self) -> dict:
        return {
            "local_frame": {
                "axis": self.local_axis,
                "lateral": self.local_lateral,
                "normal": self.local_normal,
            },
            "transverse_curvature": self.transverse_curvature,
            "axial_curvature": self.axial_curvature,
            "curvature_anisotropy": self.curvature_anisotropy,
            "radius_estimate": self.radius_estimate_m,
            "normal_axial_mad": self.normal_axial_mad_deg,
            "normal_jump": self.normal_jump_deg,
            "normal_valid_ratio": self.normal_valid_ratio,
            "object_boundary_ratio": self.object_boundary_ratio,
            "internal_cut_boundary_ratio": self.internal_cut_boundary_ratio,
            "boundary_sample_count": self.boundary_sample_count,
            "components": {
                "curvature": self.curvature_score,
                "normal_consistency": self.normal_score,
                "boundary_support": self.boundary_score,
            },
            "validity": {
                "curvature": self.curvature_valid,
                "normal": self.normal_valid,
                "boundary": self.boundary_valid,
            },
            "score_raw": self.score_raw,
            "evidence": self.evidence,
        }


@dataclass(frozen=True)
class WristGeometryBranchFeatures:
    """Explainable V2.2 shadow features; never used by V2.1 selection."""

    effective_length_m: float
    continuous_span_m: float
    width_profile_m: tuple[float | None, ...]
    width_cv: float | None
    max_width_jump: float | None
    section_stability: float
    continuity_score: float
    branching_complexity: float
    local_geometry: WristGeometryLocalGeometry
    descriptor: WristGeometryBranchDescriptor
    radius_shape: WristGeometryRadiusShape = field(
        default_factory=WristGeometryRadiusShape
    )
    topology: WristGeometryTopologyFeatures = field(
        default_factory=WristGeometryTopologyFeatures
    )
    depth_surface: WristGeometryDepthSurfaceFeatures = field(
        default_factory=WristGeometryDepthSurfaceFeatures
    )

    def diagnostics(self) -> dict:
        details = {
            "effective_length": self.effective_length_m,
            "continuous_span": self.continuous_span_m,
            "width_profile": self.width_profile_m,
            "width_cv": self.width_cv,
            "max_width_jump": self.max_width_jump,
            "section_stability": self.section_stability,
            "section_score": self.section_stability,
            "continuity_score": self.continuity_score,
            "branching_complexity": self.branching_complexity,
            "local_geometry": self.local_geometry.diagnostics(),
            "descriptor": self.descriptor.diagnostics(),
        }
        details.update(self.radius_shape.diagnostics())
        details["topology"] = self.topology.diagnostics()
        details["depth_surface"] = self.depth_surface.diagnostics()
        return details


@dataclass(frozen=True)
class WristGeometryBranch:
    """One connected component beyond the wrist dead zone."""

    branch_id: int
    point_count: int
    span_m: float
    pca_direction: ArmDirection | None
    pca_linearity: float | None
    valid_section_count: int
    inlier_section_count: int
    inlier_ratio: float
    axis: ArmDirection | None
    axis_length_m: float | None
    rms_m: float | None
    valid: bool
    reason: str
    sections: tuple[WristGeometrySection, ...] = ()
    parent_component_id: int = 0
    features: WristGeometryBranchFeatures | None = None
    local_recovery_shadow: dict | None = None


@dataclass(frozen=True)
class WristGeometryAxisEstimate:
    """Shadow-only wrist-seeded forearm-axis estimate."""

    direction: ArmDirection | None
    quality: str
    branch_count: int
    selected_branch: int | None
    axis_length_m: float | None
    rms_m: float | None
    reason: str
    branches: tuple[WristGeometryBranch, ...] = ()
    parent_topology_shadow: dict | None = None
    parent_split_shadow: dict | None = None

    def diagnostics(self) -> dict:
        return {
            "v2_axis": (
                self.direction.vector if self.direction is not None else None
            ),
            "quality": self.quality,
            "branch_count": self.branch_count,
            "selected_branch": self.selected_branch,
            "length": self.axis_length_m,
            "rms": self.rms_m,
            "failure_reason": self.reason,
        }


@dataclass
class WristGeometryTrackFeatureEma:
    """Time-smoothed forearm evidence for one persistent branch track."""

    length_m: float | None = None
    length_variance: float = 0.0
    continuity: float | None = None
    continuity_variance: float = 0.0
    width_profile_m: tuple[float | None, ...] = (None,) * 8
    width_profile_variance: tuple[float, ...] = (0.0,) * 8
    width_cv: float | None = None
    width_cv_variance: float = 0.0
    max_width_jump: float | None = None
    max_width_jump_variance: float = 0.0
    section_stability: float | None = None
    section_stability_variance: float = 0.0
    branching_complexity: float | None = None
    branching_complexity_variance: float = 0.0
    axis_delta_deg: float | None = None
    axis_delta_variance: float = 0.0
    radial_progression: float | None = None
    radial_progression_variance: float = 0.0
    radius_profile_m: tuple[float | None, ...] = (None,) * 8
    radius_profile_variance: tuple[float, ...] = (0.0,) * 8
    radius_gradient: float | None = None
    radius_gradient_variance: float = 0.0
    radius_smoothness: float | None = None
    radius_smoothness_variance: float = 0.0
    radius_jump: float | None = None
    radius_jump_variance: float = 0.0
    radius_valid_ratio: float | None = None
    radius_valid_ratio_variance: float = 0.0
    distal_branch_count: float | None = None
    distal_branch_count_variance: float = 0.0
    distal_angular_spread_deg: float | None = None
    distal_angular_spread_variance: float = 0.0
    far_shell_component_count: float | None = None
    far_shell_component_count_variance: float = 0.0
    distal_valid_ratio: float | None = None
    distal_valid_ratio_variance: float = 0.0
    wrist_bottleneck_radius_m: float | None = None
    wrist_bottleneck_radius_variance: float = 0.0
    growth_60mm: float | None = None
    growth_60mm_variance: float = 0.0
    max_local_growth: float | None = None
    max_local_growth_variance: float = 0.0
    growth_concentration: float | None = None
    growth_concentration_variance: float = 0.0
    bottleneck_ratio: float | None = None
    bottleneck_ratio_variance: float = 0.0
    bottleneck_valid_ratio: float | None = None
    bottleneck_valid_ratio_variance: float = 0.0
    surface_transverse_curvature: float | None = None
    surface_transverse_curvature_variance: float = 0.0
    surface_axial_curvature: float | None = None
    surface_axial_curvature_variance: float = 0.0
    surface_curvature_anisotropy: float | None = None
    surface_curvature_anisotropy_variance: float = 0.0
    surface_radius_estimate_m: float | None = None
    surface_radius_estimate_variance: float = 0.0
    surface_normal_axial_mad_deg: float | None = None
    surface_normal_axial_mad_variance: float = 0.0
    surface_normal_jump_deg: float | None = None
    surface_normal_jump_variance: float = 0.0
    surface_object_boundary_ratio: float | None = None
    surface_object_boundary_ratio_variance: float = 0.0
    surface_internal_cut_ratio: float | None = None
    surface_internal_cut_ratio_variance: float = 0.0
    surface_curvature_score: float | None = None
    surface_curvature_score_variance: float = 0.0
    surface_normal_score: float | None = None
    surface_normal_score_variance: float = 0.0
    surface_boundary_score: float | None = None
    surface_boundary_score_variance: float = 0.0
    depth_surface_score_raw: float | None = None
    depth_surface_score_variance: float = 0.0
    depth_surface_evidence: float | None = None
    depth_surface_evidence_variance: float = 0.0
    update_count: int = 0
    last_update_time: float | None = None

    def diagnostics(self) -> dict:
        axis_jitter = None
        if self.axis_delta_deg is not None:
            axis_jitter = self.axis_delta_deg + 2.0 * math.sqrt(
                max(0.0, self.axis_delta_variance)
            )
        width_temporal_ratios = [
            math.sqrt(max(0.0, variance)) / max(abs(float(width)), 1e-12)
            for width, variance in zip(
                self.width_profile_m, self.width_profile_variance
            )
            if width is not None
        ]
        return {
            "length": self.length_m,
            "continuity": self.continuity,
            "width_profile": self.width_profile_m,
            "width_profile_temporal_cv": (
                float(np.mean(width_temporal_ratios))
                if width_temporal_ratios else None
            ),
            "width_cv": self.width_cv,
            "max_width_jump": self.max_width_jump,
            "section_stability": self.section_stability,
            "branching_complexity": self.branching_complexity,
            "axis_delta_deg": self.axis_delta_deg,
            "axis_jitter_deg": axis_jitter,
            "radial_progression": self.radial_progression,
            "radius_profile": self.radius_profile_m,
            "radius_gradient": self.radius_gradient,
            "radius_smoothness": self.radius_smoothness,
            "radius_jump": self.radius_jump,
            "radius_valid_ratio": self.radius_valid_ratio,
            "distal_branch_count": self.distal_branch_count,
            "distal_angular_spread": self.distal_angular_spread_deg,
            "far_shell_component_count": self.far_shell_component_count,
            "distal_valid_ratio": self.distal_valid_ratio,
            "wrist_bottleneck_radius": self.wrist_bottleneck_radius_m,
            "growth_60mm": self.growth_60mm,
            "max_local_growth": self.max_local_growth,
            "growth_concentration": self.growth_concentration,
            "bottleneck_ratio": self.bottleneck_ratio,
            "bottleneck_valid_ratio": self.bottleneck_valid_ratio,
            "surface_transverse_curvature": self.surface_transverse_curvature,
            "surface_axial_curvature": self.surface_axial_curvature,
            "surface_curvature_anisotropy": self.surface_curvature_anisotropy,
            "surface_radius_estimate": self.surface_radius_estimate_m,
            "surface_normal_axial_mad": self.surface_normal_axial_mad_deg,
            "surface_normal_jump": self.surface_normal_jump_deg,
            "surface_object_boundary_ratio": self.surface_object_boundary_ratio,
            "surface_internal_cut_ratio": self.surface_internal_cut_ratio,
            "surface_curvature_score": self.surface_curvature_score,
            "surface_normal_score": self.surface_normal_score,
            "surface_boundary_score": self.surface_boundary_score,
            "depth_surface_score_raw": self.depth_surface_score_raw,
            "depth_surface_evidence": self.depth_surface_evidence,
            "update_count": self.update_count,
        }


@dataclass
class WristGeometryBranchTrack:
    """Persistent identity for a branch; shadow-only and not an output source."""

    track_id: int
    descriptor: WristGeometryBranchDescriptor
    age_frames: int = 1
    hit_count: int = 1
    consecutive_hits: int = 1
    missed_frames: int = 0
    current_branch_id: int | None = None
    last_match_cost: float | None = None
    feature_ema: WristGeometryTrackFeatureEma = field(
        default_factory=WristGeometryTrackFeatureEma
    )
    forearm_score: float = 0.0
    confidence: float = 0.0
    forearm_score_v2: float = 0.0
    confidence_v2: float = 0.0
    score_v2_components: dict = field(default_factory=dict)
    radius_components: dict = field(default_factory=dict)
    axis_delta_window_deg: deque = field(
        default_factory=lambda: deque(maxlen=12), repr=False
    )
    axis_jitter_robust_deg: float | None = None
    axis_stability_raw: float | None = None
    axis_maturity: float = 0.0
    axis_stability_effective: float = 0.5
    topology_score: float = 0.5
    topology_score_raw: float = 0.5
    topology_maturity: float = 0.0
    forearm_score_topology: float = 0.0
    topology_components: dict = field(default_factory=dict)
    last_lost_reason: str | None = None
    last_lost_detail: dict = field(default_factory=dict)
    depth_surface_score_raw: float = 0.5
    depth_surface_score_ema: float = 0.5
    depth_surface_maturity: float = 0.0
    depth_surface_score: float = 0.5
    depth_surface_components: dict = field(default_factory=dict)


@dataclass
class WristGeometryBranchTrackerState:
    """V2.2 branch association and diagnostic active-track state."""

    tracks: dict[int, WristGeometryBranchTrack] = field(default_factory=dict)
    next_track_id: int = 1
    active_track_id: int | None = None
    minimum_confirmation_frames: int = 3
    maximum_missed_frames: int = 3
    score_v2_best_track_id: int | None = None
    score_v2_best_streak_frames: int = 0
    last_lost_events: list[dict] = field(default_factory=list)


@dataclass(frozen=True)
class WristGeometryTrackedBranch:
    branch_id: int
    track_id: int | None
    match_cost: float | None
    track_age_frames: int
    consecutive_hits: int
    confirmed: bool
    valid: bool
    reason: str
    features: WristGeometryBranchFeatures | None

    def diagnostics(self) -> dict:
        details = {
            "id": self.branch_id,
            "track_id": self.track_id,
            "match_cost": self.match_cost,
            "track_age_frames": self.track_age_frames,
            "consecutive_hits": self.consecutive_hits,
            "confirmed": self.confirmed,
            "valid": self.valid,
            "reason": self.reason,
            "length": None,
            "effective_length": None,
            "continuous_span": None,
            "width_profile": (),
            "width_cv": None,
            "max_width_jump": None,
            "section_stability": 0.0,
            "section_score": 0.0,
            "continuity_score": 0.0,
            "branching_complexity": 0.0,
            "local_geometry": None,
            "descriptor": None,
        }
        if self.features is not None:
            details.update(self.features.diagnostics())
            details["length"] = self.features.effective_length_m
        return details


@dataclass(frozen=True)
class WristGeometryTrackLikelihood:
    track_id: int
    visible: bool
    age_frames: int
    consecutive_hits: int
    missed_frames: int
    ema_feature: dict
    forearm_score: float
    confidence: float
    forearm_score_v2: float
    confidence_v2: float
    score_v2_components: dict
    radius_components: dict
    axis_jitter_robust_deg: float | None
    axis_stability_raw: float | None
    axis_maturity: float
    axis_stability_effective: float
    topology_score: float
    topology_score_raw: float
    topology_maturity: float
    forearm_score_topology: float
    topology_components: dict
    track_lost_reason: str | None
    track_lost_detail: dict
    depth_surface_score_raw: float
    depth_surface_score_ema: float
    depth_surface_maturity: float
    depth_surface_score: float
    depth_surface_components: dict

    def diagnostics(self) -> dict:
        return {
            "track_id": self.track_id,
            "visible": self.visible,
            "age_frames": self.age_frames,
            "consecutive_hits": self.consecutive_hits,
            "missed_frames": self.missed_frames,
            "ema_feature": self.ema_feature,
            "forearm_score": self.forearm_score,
            "confidence": self.confidence,
            "forearm_score_v2": self.forearm_score_v2,
            "confidence_v2": self.confidence_v2,
            "score_v2_components": self.score_v2_components,
            "radius_components": self.radius_components,
            "axis_jitter_robust_deg": self.axis_jitter_robust_deg,
            "axis_stability_raw": self.axis_stability_raw,
            "axis_maturity": self.axis_maturity,
            "axis_stability_effective": self.axis_stability_effective,
            "topology_score": self.topology_score,
            "topology_score_raw": self.topology_score_raw,
            "topology_maturity": self.topology_maturity,
            "forearm_score_topology": self.forearm_score_topology,
            "forearm_score_topology_delta": (
                self.forearm_score_topology - self.forearm_score_v2
            ),
            "topology_components": self.topology_components,
            "track_lost_reason": self.track_lost_reason,
            "track_lost_detail": self.track_lost_detail,
            "depth_surface_score_raw": self.depth_surface_score_raw,
            "depth_surface_score_ema": self.depth_surface_score_ema,
            "depth_surface_maturity": self.depth_surface_maturity,
            "depth_surface_score": self.depth_surface_score,
            "depth_surface_components": self.depth_surface_components,
        }


@dataclass(frozen=True)
class WristGeometryBranchTrackingEstimate:
    branches: tuple[WristGeometryTrackedBranch, ...]
    tracks: tuple[WristGeometryTrackLikelihood, ...]
    active_track_id: int | None
    active_state: str
    candidate_track_id: int | None
    candidate_tracks: tuple[int, ...]
    score_v2_shadow: dict
    track_lost_events: tuple[dict, ...]

    def diagnostics(self) -> dict:
        return {
            "branches": [branch.diagnostics() for branch in self.branches],
            "tracks": [track.diagnostics() for track in self.tracks],
            "active_track": self.active_track_id,
            "active_track_state": self.active_state,
            "current_candidate": self.candidate_track_id,
            "candidate_tracks": self.candidate_tracks,
            "score_v2_shadow": self.score_v2_shadow,
            "track_lost_events": self.track_lost_events,
            # Step 1 deliberately observes likelihoods without selecting one.
            "active_forearm_track": None,
        }


@dataclass(frozen=True)
class ForearmAxisEstimate:
    """Selected arm-direction candidate and its source diagnostics."""

    direction: ArmDirection | None
    source: str
    axis_length_m: float | None
    rms_m: float | None
    confidence: float
    reason: str
    source_changed: bool = False
    candidate_source: str = "INVALID"
    active_source: str = "INVALID"
    selector_state: str = "WAITING_FOR_INPUT"
    switch_reason: str = "NONE"
    primary_direction: ArmDirection | None = None
    mediapipe_direction: ArmDirection | None = None
    unsigned_angle_deg: float | None = None
    axis_delta_prev_deg: float | None = None
    primary_quality_level: str = "PRIMARY_REJECTED"
    valid_section_count: int = 0
    inlier_section_count: int = 0
    inlier_ratio: float = 0.0
    max_consecutive_inlier_sections: int = 0
    max_consecutive_inlier_span_m: float = 0.0
    pca_ratio: float | None = None

    def diagnostics(self) -> dict:
        return {
            "source": self.source,
            "length": self.axis_length_m,
            "rms": self.rms_m,
            "confidence": self.confidence,
            "reason": self.reason,
            "primary_axis": (
                self.primary_direction.vector
                if self.primary_direction is not None else None
            ),
            "mediapipe_axis": (
                self.mediapipe_direction.vector
                if self.mediapipe_direction is not None else None
            ),
            "unsigned_angle": self.unsigned_angle_deg,
            "axis_delta_prev": self.axis_delta_prev_deg,
            "selector_state": self.selector_state,
            "candidate_source": self.candidate_source,
            "active_source": self.active_source,
            "switch_reason": self.switch_reason,
            "source_changed": self.source_changed,
            "primary_quality_level": self.primary_quality_level,
            "valid_section_count": self.valid_section_count,
            "inlier_section_count": self.inlier_section_count,
            "inlier_ratio": self.inlier_ratio,
            "max_consecutive_inlier_sections": (
                self.max_consecutive_inlier_sections
            ),
            "max_consecutive_inlier_span_m": (
                self.max_consecutive_inlier_span_m
            ),
            "pca_ratio": self.pca_ratio,
        }


@dataclass
class ForearmAxisSelectorState:
    """Hysteretic source selection without mixing temporal filter histories."""

    active_source: str = "INVALID"
    selector_state: str = "WAITING_FOR_INPUT"
    source_switch_count: int = 0
    last_reason: str = "NOT_INITIALIZED"
    primary_success_streak: int = 0
    primary_failure_streak: int = 0
    primary_confirm_frames: int = 3
    primary_failure_frames: int = 3
    last_primary_direction: ArmDirection | None = None


@dataclass(frozen=True)
class SurfaceNormal:
    """Oriented surface normal pointing from the camera toward the surface."""

    vector: tuple[float, float, float]
    rms_m: float
    point_count: int


@dataclass(frozen=True)
class SurfacePatch:
    """Quality-gated local skin plane at the requested image pixel."""

    point_m: tuple[float, float, float]
    normal: SurfaceNormal
    plane_inlier_count: int
    plane_residual_m: float
    normal_confidence: float
    depth_valid_ratio: float
    normal_angle_deg: float


def _landmark_pixels(landmarks, width: int, height: int) -> list[tuple[int, int]]:
    return [
        (
            int(np.clip(point.x, 0.0, 1.0) * (width - 1)),
            int(np.clip(point.y, 0.0, 1.0) * (height - 1)),
        )
        for point in landmarks
    ]


def pulse_pixel(landmarks, width: int, height: int) -> tuple[int, int]:
    """Place a target just proximal to the wrist on its anatomical thumb side.

    MediaPipe landmark indices identify the thumb side directly, so this works
    for either hand without relying on a possibly mirrored handedness label.
    The location is an interaction region, not a medical landmark diagnosis.
    """
    points = np.asarray(_landmark_pixels(landmarks, width, height), dtype=np.float64)
    wrist = points[0]
    palm_center = np.mean(points[[5, 9, 13, 17]], axis=0)
    palm_to_forearm = wrist - palm_center
    longitudinal_norm = float(np.linalg.norm(palm_to_forearm))
    if longitudinal_norm < 1.0:
        raise ValueError("hand landmarks do not define a usable wrist direction")
    palm_to_forearm /= longitudinal_norm

    # Index MCP (5) is on the anatomical radial/thumb side and pinky MCP (17)
    # is on the ulnar side. Move slightly toward the thumb and slightly onto
    # the forearm so the marker does not sit in the palm.
    radial = points[5] - points[17]
    palm_width = float(np.linalg.norm(radial))
    if palm_width < 8.0:
        raise ValueError("hand landmarks do not define a usable palm width")
    radial /= palm_width
    target = wrist + 0.18 * palm_width * palm_to_forearm + 0.20 * palm_width * radial
    return (
        int(np.clip(round(target[0]), 0, width - 1)),
        int(np.clip(round(target[1]), 0, height - 1)),
    )


def median_depth_at(
    depth_m: np.ndarray,
    pixel: tuple[int, int],
    radius_px: int,
    minimum_m: float = 0.10,
    maximum_m: float = 2.0,
) -> float | None:
    """Return a robust local depth after rejecting invalid and distant pixels."""
    height, width = depth_m.shape
    x, y = pixel
    radius_px = max(1, int(radius_px))
    x0, x1 = max(0, x - radius_px), min(width, x + radius_px + 1)
    y0, y1 = max(0, y - radius_px), min(height, y + radius_px + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[
        np.isfinite(patch) & (patch >= minimum_m) & (patch <= maximum_m)
    ]
    if valid.size < 5:
        return None
    median = float(np.median(valid))
    close = valid[np.abs(valid - median) <= 0.025]
    return float(np.median(close)) if close.size >= 5 else None


def deproject_pixel(
    pixel: tuple[int, int],
    depth_m: float,
    intrinsics: CameraIntrinsics,
    image_width: int,
    image_height: int,
    mirrored: bool = False,
) -> tuple[float, float, float]:
    """Deproject a registered color pixel into its optical camera frame."""
    u, v = pixel
    if mirrored:
        u = image_width - 1 - u

    scale_x = intrinsics.width / float(image_width)
    scale_y = intrinsics.height / float(image_height)
    u_info = (u + 0.5) * scale_x - 0.5
    v_info = (v + 0.5) * scale_y - 0.5
    return (
        (u_info - intrinsics.cx) * depth_m / intrinsics.fx,
        (v_info - intrinsics.cy) * depth_m / intrinsics.fy,
        depth_m,
    )


def estimate_pulse_region(
    landmarks,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
) -> PulseRegion | None:
    """Estimate one approximate pulse region from a single registered frame."""
    region, _reason, _details = estimate_pulse_region_diagnostic(
        landmarks, depth_m, intrinsics, mirrored
    )
    return region


def estimate_pulse_region_diagnostic(
    landmarks,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
) -> tuple[PulseRegion | None, str, dict]:
    """Estimate a region and preserve the exact rejection stage for diagnostics."""
    height, width = depth_m.shape
    try:
        pixel = pulse_pixel(landmarks, width, height)
    except ValueError as error:
        return None, "INVALID_3D_POINT", {"detail": str(error)}
    points = _landmark_pixels(landmarks, width, height)
    palm_width_px = max(8, int(np.linalg.norm(np.asarray(points[5]) - points[17])))
    depth = median_depth_at(depth_m, pixel, max(3, palm_width_px // 9))
    if depth is None:
        return None, "INVALID_DEPTH", {"raw_pixel_uv": pixel, "raw_depth": None}
    point = deproject_pixel(
        pixel, depth, intrinsics, width, height, mirrored=mirrored
    )
    if not all(math.isfinite(value) for value in point):
        return None, "INVALID_3D_POINT", {
            "raw_pixel_uv": pixel, "raw_depth": depth, "camera_xyz": point,
        }
    radius_m = max(0.006, min(0.018, depth * palm_width_px / intrinsics.fx * 0.18))
    return (
        PulseRegion(pixel=pixel, point_m=point, radius_m=radius_m),
        "NONE",
        {"raw_pixel_uv": pixel, "raw_depth": depth, "camera_xyz": point},
    )


def estimate_arm_direction(
    landmarks,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
) -> ArmDirection | None:
    """Estimate a 3D forearm axis from palm center toward the wrist.

    MediaPipe has no forearm landmark. This is therefore an approximate axis
    continuing the hand longitudinal direction, with registered depth at both
    endpoints. Callers must treat d and -d as the same anatomical axis.
    """
    height, width = depth_m.shape
    points = _landmark_pixels(landmarks, width, height)
    wrist = points[0]
    palm = tuple(
        int(round(value))
        for value in np.mean(np.asarray([points[5], points[9], points[13], points[17]]), axis=0)
    )
    palm_width = max(8, int(np.linalg.norm(np.asarray(points[5]) - points[17])))
    radius = max(3, palm_width // 9)
    wrist_depth = median_depth_at(depth_m, wrist, radius)
    palm_depth = median_depth_at(depth_m, palm, radius)
    if wrist_depth is None or palm_depth is None:
        return None
    wrist_xyz = np.asarray(
        deproject_pixel(wrist, wrist_depth, intrinsics, width, height, mirrored),
        dtype=np.float64,
    )
    palm_xyz = np.asarray(
        deproject_pixel(palm, palm_depth, intrinsics, width, height, mirrored),
        dtype=np.float64,
    )
    vector = wrist_xyz - palm_xyz
    length = float(np.linalg.norm(vector))
    if not np.isfinite(length) or length < 0.015:
        return None
    vector /= length
    return ArmDirection(tuple(float(value) for value in vector))


def masked_depth_to_points(
    arm_mask: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
    minimum_depth_m: float = 0.10,
    maximum_depth_m: float = 2.0,
    maximum_points: int = 5000,
) -> tuple[np.ndarray, np.ndarray]:
    """Deproject valid masked depth into camera points and image pixels."""
    if arm_mask.shape != depth_m.shape:
        raise ValueError("forearm mask must match the depth image")
    valid = (
        (arm_mask > 0)
        & np.isfinite(depth_m)
        & (depth_m >= minimum_depth_m)
        & (depth_m <= maximum_depth_m)
    )
    rows, cols = np.nonzero(valid)
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 2), dtype=np.float64)
    if maximum_points > 0 and rows.size > maximum_points:
        indices = np.linspace(0, rows.size - 1, maximum_points, dtype=np.int64)
        rows, cols = rows[indices], cols[indices]

    height, width = depth_m.shape
    depth = depth_m[rows, cols].astype(np.float64)
    deprojection_cols = width - 1 - cols if mirrored else cols
    scale_x = intrinsics.width / float(width)
    scale_y = intrinsics.height / float(height)
    u_info = (deprojection_cols.astype(np.float64) + 0.5) * scale_x - 0.5
    v_info = (rows.astype(np.float64) + 0.5) * scale_y - 0.5
    points = np.column_stack((
        (u_info - intrinsics.cx) * depth / intrinsics.fx,
        (v_info - intrinsics.cy) * depth / intrinsics.fy,
        depth,
    ))
    pixels = np.column_stack((cols.astype(np.float64), rows.astype(np.float64)))
    finite = np.all(np.isfinite(points), axis=1)
    return points[finite], pixels[finite]


def select_forearm_corridor(
    arm_mask: np.ndarray,
    landmarks,
) -> tuple[np.ndarray, tuple[float, float], tuple[float, float], float]:
    """Keep the wrist-connected forearm corridor defined by hand geometry."""
    height, width = arm_mask.shape
    landmark_pixels = np.asarray(
        _landmark_pixels(landmarks, width, height), dtype=np.float64
    )
    wrist = landmark_pixels[0]
    palm_center = np.mean(landmark_pixels[[5, 9, 13, 17]], axis=0)
    image_axis = wrist - palm_center
    axis_norm = float(np.linalg.norm(image_axis))
    palm_width_px = float(np.linalg.norm(landmark_pixels[5] - landmark_pixels[17]))
    if axis_norm < 1.0 or palm_width_px < 8.0:
        return (
            np.zeros_like(arm_mask, dtype=np.uint8),
            tuple(float(value) for value in wrist),
            (0.0, 0.0),
            palm_width_px,
        )
    image_axis /= axis_norm
    rows, cols = np.indices(arm_mask.shape, dtype=np.float64)
    delta_x = cols - wrist[0]
    delta_y = rows - wrist[1]
    longitudinal = delta_x * image_axis[0] + delta_y * image_axis[1]
    lateral = np.abs(-delta_x * image_axis[1] + delta_y * image_axis[0])
    minimum_longitudinal = 0.12 * palm_width_px
    maximum_longitudinal = 4.4 * palm_width_px
    taper = np.clip(longitudinal / max(maximum_longitudinal, 1.0), 0.0, 1.0)
    half_width = palm_width_px * (0.66 + 0.36 * taper)
    corridor = (
        (arm_mask > 0)
        & (longitudinal >= minimum_longitudinal)
        & (longitudinal <= maximum_longitudinal)
        & (lateral <= half_width)
    )
    return (
        np.uint8(corridor) * 255,
        tuple(float(value) for value in wrist),
        tuple(float(value) for value in image_axis),
        palm_width_px,
    )


def compute_forearm_sections(
    points_m: np.ndarray,
    pixels: np.ndarray,
    wrist_pixel: tuple[float, float],
    image_axis: tuple[float, float],
    section_count: int = 9,
    minimum_points_per_section: int = 20,
) -> list[ForearmSection]:
    """Reduce the forearm surface to indexed longitudinal cross-sections."""
    if points_m.ndim != 2 or points_m.shape[1:] != (3,):
        raise ValueError("forearm points must have shape Nx3")
    if pixels.shape != (points_m.shape[0], 2):
        raise ValueError("forearm pixels must have shape Nx2")
    if points_m.shape[0] < minimum_points_per_section or section_count < 3:
        return []
    axis = np.asarray(image_axis, dtype=np.float64)
    axis_length = float(np.linalg.norm(axis))
    if not math.isfinite(axis_length) or axis_length < 1e-6:
        return []
    axis /= axis_length
    longitudinal = (pixels - np.asarray(wrist_pixel, dtype=np.float64)) @ axis
    lower, upper = np.percentile(longitudinal, (2.0, 98.0))
    if not np.isfinite(lower + upper) or upper - lower < 3.0:
        return []
    edges = np.linspace(lower, upper, section_count + 1)
    sections: list[ForearmSection] = []
    for index in range(section_count):
        selected = (longitudinal >= edges[index]) & (
            longitudinal <= edges[index + 1]
            if index == section_count - 1
            else longitudinal < edges[index + 1]
        )
        section_points = points_m[selected]
        if section_points.shape[0] < minimum_points_per_section:
            sections.append(ForearmSection(
                section_index=index,
                center_m=None,
                longitudinal_px=float((edges[index] + edges[index + 1]) * 0.5),
                point_count=int(section_points.shape[0]),
                radius_m=None,
                depth_spread_m=None,
                valid=False,
            ))
            continue
        center = np.median(section_points, axis=0)
        distances = np.linalg.norm(section_points - center, axis=1)
        depth_deviation = np.abs(section_points[:, 2] - np.median(section_points[:, 2]))
        sections.append(ForearmSection(
            section_index=index,
            center_m=tuple(float(value) for value in center),
            longitudinal_px=float(np.median(longitudinal[selected])),
            point_count=int(section_points.shape[0]),
            radius_m=float(np.median(distances)),
            depth_spread_m=float(1.4826 * np.median(depth_deviation)),
            valid=True,
        ))
    return sections


def fit_forearm_centerline_ransac(
    sections: list[ForearmSection],
    distance_threshold_m: float = 0.006,
    minimum_inlier_sections: int = 4,
) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray, str]:
    """Deterministically RANSAC-fit a line to cross-section centers."""
    valid_sections = [
        section for section in sections
        if section.valid and section.center_m is not None
    ]
    if len(valid_sections) < minimum_inlier_sections:
        return None, None, np.empty(0, dtype=np.int64), "TOO_FEW_SECTIONS"
    centers = np.asarray(
        [section.center_m for section in valid_sections], dtype=np.float64
    )
    if not np.all(np.isfinite(centers)):
        return None, None, np.empty(0, dtype=np.int64), "NONFINITE"
    best_indices = np.empty(0, dtype=np.int64)
    best_rms = math.inf
    best_origin = None
    best_direction = None
    for first in range(len(valid_sections) - 1):
        for second in range(first + 1, len(valid_sections)):
            direction = centers[second] - centers[first]
            length = float(np.linalg.norm(direction))
            if not math.isfinite(length) or length < 0.015:
                continue
            direction /= length
            delta = centers - centers[first]
            residuals = np.linalg.norm(delta - np.outer(delta @ direction, direction), axis=1)
            indices = np.flatnonzero(residuals <= distance_threshold_m)
            rms = (
                float(math.sqrt(np.mean(np.square(residuals[indices]))))
                if indices.size else math.inf
            )
            if indices.size > best_indices.size or (
                indices.size == best_indices.size and rms < best_rms
            ):
                best_indices = np.asarray(
                    [valid_sections[item].section_index for item in indices],
                    dtype=np.int64,
                )
                best_rms = rms
                best_origin = centers[first]
                best_direction = direction
    if best_indices.size < minimum_inlier_sections:
        return None, None, best_indices, "RANSAC_FAILED"
    return best_origin, best_direction, best_indices, "NONE"


def _wrist_radius_shape(
    axial_positions: list[float],
    radii_m: list[float | None],
) -> WristGeometryRadiusShape:
    """Summarize a section radius profile without fitting a cylinder."""
    valid = [
        (float(position), float(radius))
        for position, radius in zip(axial_positions, radii_m)
        if radius is not None and math.isfinite(radius) and radius > 0.0
    ]
    valid_ratio = len(valid) / max(1, len(radii_m))
    if not valid:
        return WristGeometryRadiusShape(valid_ratio=valid_ratio)

    positions = np.asarray([item[0] for item in valid], dtype=np.float64)
    radii = np.asarray([item[1] for item in valid], dtype=np.float64)
    order = np.argsort(positions)
    positions = positions[order]
    radii = radii[order]
    if radii.size == 1 or positions[-1] - positions[0] <= 1e-9:
        profile = tuple(float(radii[0]) for _ in range(8))
        return WristGeometryRadiusShape(
            profile_m=profile,
            valid_ratio=valid_ratio,
        )

    sample_positions = np.linspace(positions[0], positions[-1], 8)
    profile = tuple(float(value) for value in np.interp(
        sample_positions, positions, radii
    ))
    median_radius = max(float(np.median(radii)), 1e-12)
    slopes = [
        float((radii[second] - radii[first]) /
              (positions[second] - positions[first]))
        for first in range(radii.size - 1)
        for second in range(first + 1, radii.size)
        if positions[second] - positions[first] > 1e-9
    ]
    if not slopes:
        return WristGeometryRadiusShape(
            profile_m=profile,
            valid_ratio=valid_ratio,
        )
    slope = float(np.median(slopes))
    intercept = float(np.median(radii - slope * positions))
    residuals = radii - (intercept + slope * positions)
    normalized_rms = float(math.sqrt(np.mean(np.square(residuals)))) / median_radius
    span = float(positions[-1] - positions[0])
    gradient = slope * span / median_radius
    smoothness = math.exp(-((normalized_rms / 0.20) ** 2))
    jumps = np.abs(np.diff(radii)) / median_radius
    jump = float(np.max(jumps)) if jumps.size else 0.0
    return WristGeometryRadiusShape(
        profile_m=profile,
        gradient=float(gradient),
        smoothness=float(np.clip(smoothness, 0.0, 1.0)),
        jump=jump,
        valid_ratio=valid_ratio,
    )


def _topology_component_count(
    points: np.ndarray,
    pixels: np.ndarray,
    graph_stride: int,
) -> int:
    """Count substantial image-connected components in one 3D shell."""
    if points.shape[0] < 12 or pixels.shape != (points.shape[0], 2):
        return 0
    coordinates = {
        (int(pixel[0]), int(pixel[1])): index
        for index, pixel in enumerate(pixels)
    }
    visited = np.zeros(points.shape[0], dtype=bool)
    component_sizes: list[int] = []
    offsets = (
        (-graph_stride, -graph_stride), (0, -graph_stride),
        (graph_stride, -graph_stride), (-graph_stride, 0),
        (graph_stride, 0), (-graph_stride, graph_stride),
        (0, graph_stride), (graph_stride, graph_stride),
    )
    for start in range(points.shape[0]):
        if visited[start]:
            continue
        visited[start] = True
        queue = deque([start])
        size = 0
        while queue:
            index = queue.popleft()
            size += 1
            column, row = (int(value) for value in pixels[index])
            for dx, dy in offsets:
                neighbor = coordinates.get((column + dx, row + dy))
                if neighbor is None or visited[neighbor]:
                    continue
                edge_limit = max(
                    0.006,
                    0.012 * min(float(points[index, 2]), float(points[neighbor, 2])),
                )
                if float(np.linalg.norm(points[neighbor] - points[index])) > edge_limit:
                    continue
                visited[neighbor] = True
                queue.append(neighbor)
        component_sizes.append(size)
    minimum_size = max(6, int(math.ceil(0.05 * points.shape[0])))
    return sum(size >= minimum_size for size in component_sizes)


def _extract_wrist_topology_features(
    parent_points: np.ndarray | None,
    parent_pixels: np.ndarray | None,
    wrist_point: np.ndarray,
    axis: np.ndarray,
    branch_length_m: float,
    graph_stride: int,
) -> WristGeometryTopologyFeatures:
    """Measure parent-cloud fan-out and near-wrist expansion for one branch."""
    if (
        parent_points is None
        or parent_pixels is None
        or parent_points.shape[0] < 40
        or parent_pixels.shape != (parent_points.shape[0], 2)
        or branch_length_m <= 0.0
    ):
        return WristGeometryTopologyFeatures()
    direction = np.asarray(axis, dtype=np.float64)
    direction /= max(float(np.linalg.norm(direction)), 1e-12)
    relative = parent_points - wrist_point
    distance = np.linalg.norm(relative, axis=1)
    unit = relative / np.maximum(distance[:, None], 1e-12)
    axial = relative @ direction
    cosine = np.clip(unit @ direction, -1.0, 1.0)
    angle_deg = np.degrees(np.arccos(cosine))
    lateral = np.linalg.norm(relative - np.outer(axial, direction), axis=1)

    distal_start = max(0.060, 0.55 * branch_length_m)
    distal_end = min(0.25, max(distal_start + 0.025, 1.05 * branch_length_m))
    distal_mask = (
        (axial >= distal_start)
        & (axial <= distal_end)
        & (angle_deg <= 75.0)
    )
    distal_indices = np.flatnonzero(distal_mask)
    distal_branch_count = None
    distal_spread = None
    far_component_count = None
    distal_valid = distal_indices.size >= 20
    if distal_valid:
        distal_directions = unit[distal_indices]
        ordered = distal_indices[np.argsort(distance[distal_indices])[::-1]]
        cluster_sums: list[np.ndarray] = []
        cluster_sizes: list[int] = []
        # A 30-degree cone keeps the two visible sides of a thick cylindrical
        # forearm in one lobe while still separating genuinely diverging tips.
        cluster_cosine = math.cos(math.radians(30.0))
        for point_index in ordered:
            candidate = unit[point_index]
            if not cluster_sums:
                cluster_sums.append(candidate.copy())
                cluster_sizes.append(1)
                continue
            centers = [
                value / max(float(np.linalg.norm(value)), 1e-12)
                for value in cluster_sums
            ]
            similarities = [float(np.dot(candidate, center)) for center in centers]
            selected = int(np.argmax(similarities))
            if similarities[selected] >= cluster_cosine:
                cluster_sums[selected] += candidate
                cluster_sizes[selected] += 1
            elif len(cluster_sums) < 8:
                cluster_sums.append(candidate.copy())
                cluster_sizes.append(1)
        minimum_cluster = max(8, int(math.ceil(0.05 * distal_indices.size)))
        distal_branch_count = max(
            1, sum(size >= minimum_cluster for size in cluster_sizes)
        )
        distal_angles = angle_deg[distal_indices]
        distal_spread = float(
            np.percentile(distal_angles, 90.0)
            - np.percentile(distal_angles, 10.0)
        )
        far_start = max(0.080, 0.75 * branch_length_m)
        far_mask = (
            (axial >= far_start)
            & (axial <= distal_end + 0.015)
            & (angle_deg <= 75.0)
        )
        far_component_count = _topology_component_count(
            parent_points[far_mask], parent_pixels[far_mask], graph_stride
        )
        if far_component_count == 0 and int(np.count_nonzero(far_mask)) >= 12:
            far_component_count = 1

    bottleneck_edges = np.asarray((0.025, 0.037, 0.049, 0.061, 0.073, 0.085))
    radii: list[float | None] = []
    for lower, upper in zip(bottleneck_edges[:-1], bottleneck_edges[1:]):
        selected = (
            (axial >= lower)
            & (axial < upper)
            & (angle_deg <= 60.0)
            & (lateral <= 0.080)
        )
        section = parent_points[selected]
        if section.shape[0] < 8:
            radii.append(None)
            continue
        center = np.median(section, axis=0)
        centered = section - center
        transverse = centered - np.outer(centered @ direction, direction)
        radius = float(np.percentile(np.linalg.norm(transverse, axis=1), 80.0))
        radii.append(radius if math.isfinite(radius) and radius > 0.0 else None)
    valid_radii = [(index, value) for index, value in enumerate(radii) if value is not None]
    bottleneck_valid = len(valid_radii) >= 4 and radii[0] is not None
    bottleneck_radius = growth_60mm = maximum_growth = concentration = ratio = None
    if bottleneck_valid:
        bottleneck_radius = float(radii[0])
        last_radius = float(valid_radii[-1][1])
        growth_60mm = (last_radius - bottleneck_radius) / max(
            bottleneck_radius, 1e-12
        )
        local_growth = [
            (float(second) - float(first)) / max(float(first), 1e-12)
            for first, second in zip(radii, radii[1:])
            if first is not None and second is not None
        ]
        positive_growth = [max(0.0, value) for value in local_growth]
        maximum_growth = max(positive_growth, default=0.0)
        positive_sum = sum(positive_growth)
        concentration = (
            maximum_growth / positive_sum if positive_sum > 1e-12 else 0.0
        )
        ratio = max(float(value) for _, value in valid_radii) / max(
            bottleneck_radius, 1e-12
        )
    return WristGeometryTopologyFeatures(
        distal_branch_count=distal_branch_count,
        distal_angular_spread_deg=distal_spread,
        far_shell_component_count=far_component_count,
        distal_valid=distal_valid,
        wrist_bottleneck_radius_m=bottleneck_radius,
        growth_60mm=growth_60mm,
        max_local_growth=maximum_growth,
        growth_concentration=concentration,
        bottleneck_ratio=ratio,
        bottleneck_valid=bottleneck_valid,
    )


def _extract_wrist_depth_surface_features(
    branch_points: np.ndarray,
    branch_pixels: np.ndarray | None,
    parent_points: np.ndarray | None,
    parent_pixels: np.ndarray | None,
    wrist_point: np.ndarray,
    axis: np.ndarray,
    graph_stride: int,
) -> WristGeometryDepthSurfaceFeatures:
    """Extract branch surface evidence without feeding any selection score."""
    if branch_points.shape[0] < 40:
        return WristGeometryDepthSurfaceFeatures()

    local_axis = np.asarray(axis, dtype=np.float64)
    local_axis /= max(float(np.linalg.norm(local_axis)), 1e-12)
    center = np.median(branch_points, axis=0)
    view_direction = center / max(float(np.linalg.norm(center)), 1e-12)
    local_lateral = np.cross(local_axis, view_direction)
    lateral_norm = float(np.linalg.norm(local_lateral))
    if lateral_norm < 0.10:
        fallback = np.asarray((1.0, 0.0, 0.0), dtype=np.float64)
        if abs(float(np.dot(fallback, local_axis))) > 0.85:
            fallback = np.asarray((0.0, 1.0, 0.0), dtype=np.float64)
        local_lateral = np.cross(local_axis, fallback)
        lateral_norm = float(np.linalg.norm(local_lateral))
    local_lateral /= max(lateral_norm, 1e-12)
    local_normal = np.cross(local_axis, local_lateral)
    local_normal /= max(float(np.linalg.norm(local_normal)), 1e-12)
    if float(np.dot(local_normal, -center)) < 0.0:
        local_lateral = -local_lateral
        local_normal = -local_normal

    centered = branch_points - center
    axial = centered @ local_axis
    transverse = centered @ local_lateral
    height = centered @ local_normal

    transverse_curvature = axial_curvature = anisotropy = radius_estimate = None
    curvature_score = 0.5
    curvature_valid = False
    design = np.column_stack((
        np.ones(branch_points.shape[0]),
        axial,
        transverse,
        0.5 * axial * axial,
        axial * transverse,
        0.5 * transverse * transverse,
    ))
    fit_mask = np.ones(branch_points.shape[0], dtype=bool)
    coefficients = None
    for _ in range(3):
        if int(np.count_nonzero(fit_mask)) < 30:
            break
        coefficients, *_ = np.linalg.lstsq(
            design[fit_mask], height[fit_mask], rcond=None
        )
        residual = height - design @ coefficients
        residual_center = float(np.median(residual[fit_mask]))
        residual_mad = float(np.median(
            np.abs(residual[fit_mask] - residual_center)
        ))
        threshold = max(0.0025, 3.0 * 1.4826 * residual_mad)
        fit_mask = np.abs(residual - residual_center) <= threshold
    if coefficients is not None and int(np.count_nonzero(fit_mask)) >= 30:
        axial_curvature = float(abs(coefficients[3]))
        transverse_curvature = float(abs(coefficients[5]))
        curvature_sum = axial_curvature + transverse_curvature
        anisotropy = float(
            transverse_curvature / max(curvature_sum, 1e-12)
        )
        if transverse_curvature > 1e-3:
            radius_estimate = float(1.0 / transverse_curvature)
        magnitude_score = _bounded_plateau_score(
            transverse_curvature, 3.0, 12.0, 80.0, 140.0
        )
        radius_score = _bounded_plateau_score(
            radius_estimate, 0.008, 0.015, 0.080, 0.140
        )
        curvature_score = float(
            max(anisotropy, 1e-12) ** 0.35
            * max(magnitude_score, 1e-12) ** 0.35
            * max(radius_score, 1e-12) ** 0.30
        )
        curvature_valid = bool(
            np.all(np.isfinite(coefficients))
            and np.count_nonzero(fit_mask) >= max(30, branch_points.shape[0] // 3)
        )

    normal_mad = normal_jump = None
    normal_valid_ratio = 0.0
    normal_score = 0.5
    normal_valid = False
    if (
        branch_pixels is not None
        and branch_pixels.shape == (branch_points.shape[0], 2)
    ):
        pixel_to_index = {
            (int(pixel[0]), int(pixel[1])): index
            for index, pixel in enumerate(branch_pixels)
        }
        sampled_normals: list[np.ndarray] = []
        sampled_axial: list[float] = []
        sampled_transverse: list[float] = []
        stride = max(1, int(graph_stride))
        for index, pixel in enumerate(branch_pixels):
            column, row = int(pixel[0]), int(pixel[1])
            left = pixel_to_index.get((column - stride, row))
            right = pixel_to_index.get((column + stride, row))
            up = pixel_to_index.get((column, row - stride))
            down = pixel_to_index.get((column, row + stride))
            if None in (left, right, up, down):
                continue
            horizontal = branch_points[right] - branch_points[left]
            vertical = branch_points[down] - branch_points[up]
            normal = np.cross(horizontal, vertical)
            normal_norm = float(np.linalg.norm(normal))
            if normal_norm <= 1e-10:
                continue
            normal /= normal_norm
            if float(np.dot(normal, -branch_points[index])) < 0.0:
                normal = -normal
            sampled_normals.append(normal)
            sampled_axial.append(float(axial[index]))
            sampled_transverse.append(float(transverse[index]))
        normal_valid_ratio = len(sampled_normals) / max(1, branch_points.shape[0])
        if len(sampled_normals) >= 24:
            normal_array = np.asarray(sampled_normals)
            normal_axial = np.asarray(sampled_axial)
            normal_transverse = np.asarray(sampled_transverse)
            axial_edges = np.linspace(
                *np.percentile(normal_axial, (2.0, 98.0)), 7
            )
            transverse_edges = np.linspace(
                *np.percentile(normal_transverse, (5.0, 95.0)), 4
            )
            section_normals: dict[tuple[int, int], np.ndarray] = {}
            for section_index in range(6):
                for lateral_index in range(3):
                    selected = (
                        (normal_axial >= axial_edges[section_index])
                        & (normal_axial <= axial_edges[section_index + 1])
                        & (normal_transverse >= transverse_edges[lateral_index])
                        & (normal_transverse <= transverse_edges[lateral_index + 1])
                    )
                    if int(np.count_nonzero(selected)) < 3:
                        continue
                    section_normal = np.median(normal_array[selected], axis=0)
                    section_normal /= max(
                        float(np.linalg.norm(section_normal)), 1e-12
                    )
                    section_normals[(section_index, lateral_index)] = section_normal
            axial_angles = []
            for section_index in range(5):
                for lateral_index in range(3):
                    first = section_normals.get((section_index, lateral_index))
                    second = section_normals.get((section_index + 1, lateral_index))
                    if first is None or second is None:
                        continue
                    cosine = float(np.clip(np.dot(first, second), -1.0, 1.0))
                    axial_angles.append(float(np.degrees(math.acos(cosine))))
            if len(axial_angles) >= 4:
                angle_array = np.asarray(axial_angles)
                angle_median = float(np.median(angle_array))
                normal_mad = float(
                    1.4826 * np.median(np.abs(angle_array - angle_median))
                )
                normal_jump = float(np.percentile(angle_array, 95.0))
                normal_score = float(math.sqrt(
                    math.exp(-((normal_mad / 8.0) ** 2))
                    * math.exp(-((normal_jump / 20.0) ** 2))
                ))
                normal_valid = True

    object_boundary_ratio = internal_cut_ratio = None
    boundary_sample_count = 0
    boundary_score = 0.5
    boundary_valid = False
    if (
        branch_pixels is not None
        and branch_pixels.shape == (branch_points.shape[0], 2)
        and parent_pixels is not None
        and parent_points is not None
        and parent_pixels.shape == (parent_points.shape[0], 2)
    ):
        branch_pixel_set = {
            (int(pixel[0]), int(pixel[1])) for pixel in branch_pixels
        }
        parent_pixel_set = {
            (int(pixel[0]), int(pixel[1])) for pixel in parent_pixels
        }
        wrist_axial = (branch_points - wrist_point) @ local_axis
        positive_axial = wrist_axial[wrist_axial > 0.025]
        far_limit = (
            float(np.percentile(positive_axial, 90.0))
            if positive_axial.size else math.inf
        )
        object_edges = internal_edges = 0
        stride = max(1, int(graph_stride))
        for pixel, point_axial in zip(branch_pixels, wrist_axial):
            if point_axial < 0.025 or point_axial > far_limit:
                continue
            column, row = int(pixel[0]), int(pixel[1])
            for neighbor in (
                (column - stride, row), (column + stride, row),
                (column, row - stride), (column, row + stride),
            ):
                if neighbor in branch_pixel_set:
                    continue
                if neighbor in parent_pixel_set:
                    internal_edges += 1
                else:
                    object_edges += 1
        boundary_sample_count = object_edges + internal_edges
        if boundary_sample_count >= 12:
            object_boundary_ratio = object_edges / boundary_sample_count
            internal_cut_ratio = internal_edges / boundary_sample_count
            boundary_score = float(object_boundary_ratio)
            boundary_valid = True

    component_scores = (
        curvature_score if curvature_valid else 0.5,
        normal_score if normal_valid else 0.5,
        boundary_score if boundary_valid else 0.5,
    )
    score_raw = float(
        0.45 * component_scores[0]
        + 0.30 * component_scores[1]
        + 0.25 * component_scores[2]
    )
    normal_evidence = (
        min(1.0, normal_valid_ratio / 0.30) if normal_valid else 0.0
    )
    evidence = float(
        (float(curvature_valid) + normal_evidence + float(boundary_valid)) / 3.0
    )
    return WristGeometryDepthSurfaceFeatures(
        local_axis=tuple(float(value) for value in local_axis),
        local_lateral=tuple(float(value) for value in local_lateral),
        local_normal=tuple(float(value) for value in local_normal),
        transverse_curvature=transverse_curvature,
        axial_curvature=axial_curvature,
        curvature_anisotropy=anisotropy,
        radius_estimate_m=radius_estimate,
        curvature_score=float(np.clip(curvature_score, 0.0, 1.0)),
        curvature_valid=curvature_valid,
        normal_axial_mad_deg=normal_mad,
        normal_jump_deg=normal_jump,
        normal_valid_ratio=float(normal_valid_ratio),
        normal_score=float(np.clip(normal_score, 0.0, 1.0)),
        normal_valid=normal_valid,
        object_boundary_ratio=object_boundary_ratio,
        internal_cut_boundary_ratio=internal_cut_ratio,
        boundary_sample_count=boundary_sample_count,
        boundary_score=float(np.clip(boundary_score, 0.0, 1.0)),
        boundary_valid=boundary_valid,
        score_raw=float(np.clip(score_raw, 0.0, 1.0)),
        evidence=float(np.clip(evidence, 0.0, 1.0)),
    )


def _extract_wrist_branch_features(
    branch_points: np.ndarray,
    wrist_point: np.ndarray,
    pca_axis: np.ndarray | None,
    eigenvalues: np.ndarray | None,
    sibling_count: int,
    branch_pixels: np.ndarray | None = None,
    parent_points: np.ndarray | None = None,
    parent_pixels: np.ndarray | None = None,
    graph_stride: int = 1,
) -> WristGeometryBranchFeatures:
    """Extract V2.2 diagnostics without changing V2.1 branch validity."""
    radial_distance = np.linalg.norm(branch_points - wrist_point, axis=1)
    radial_span = (
        float(np.max(radial_distance) - np.min(radial_distance))
        if radial_distance.size else 0.0
    )
    relative_centroid = np.mean(branch_points, axis=0) - wrist_point
    empty_local = WristGeometryLocalGeometry(None, None, None, None)
    empty_descriptor = WristGeometryBranchDescriptor(
        axis=None,
        centroid_m=tuple(float(value) for value in relative_centroid),
        near_endpoint_m=tuple(float(value) for value in relative_centroid),
        far_endpoint_m=tuple(float(value) for value in relative_centroid),
        effective_length_m=radial_span,
        median_width_m=None,
    )
    if pca_axis is None or not np.all(np.isfinite(pca_axis)):
        return WristGeometryBranchFeatures(
            effective_length_m=radial_span,
            continuous_span_m=0.0,
            width_profile_m=(),
            width_cv=None,
            max_width_jump=None,
            section_stability=0.0,
            continuity_score=0.0,
            branching_complexity=float(np.clip((sibling_count - 1) / 3.0, 0.0, 1.0)),
            local_geometry=empty_local,
            descriptor=empty_descriptor,
        )

    axis = np.asarray(pca_axis, dtype=np.float64)
    axis /= max(float(np.linalg.norm(axis)), 1e-12)
    relative = branch_points - wrist_point
    axial = relative @ axis
    lower, upper = np.percentile(axial, (2.0, 98.0))
    axial_span = max(0.0, float(upper - lower))
    effective_length = min(radial_span, axial_span) if axial_span > 0.0 else radial_span
    section_count = int(np.clip(math.ceil(max(axial_span, 0.001) / 0.020), 5, 12))
    edges = np.linspace(lower, upper, section_count + 1)
    minimum_points = max(8, int(branch_points.shape[0] / section_count * 0.10))
    width_profile: list[float | None] = []
    radius_profile: list[float | None] = []
    radius_positions: list[float] = []
    section_centers: list[np.ndarray | None] = []
    section_counts: list[int] = []
    for section_index in range(section_count):
        selected = (axial >= edges[section_index]) & (
            axial <= edges[section_index + 1]
            if section_index == section_count - 1
            else axial < edges[section_index + 1]
        )
        section_points = branch_points[selected]
        radius_positions.append(float(
            (edges[section_index] + edges[section_index + 1]) * 0.5
        ))
        section_counts.append(int(section_points.shape[0]))
        if section_points.shape[0] < minimum_points:
            width_profile.append(None)
            radius_profile.append(None)
            section_centers.append(None)
            continue
        center = np.median(section_points, axis=0)
        section_centers.append(center)
        centered = section_points - center
        transverse = centered - np.outer(centered @ axis, axis)
        transverse_covariance = transverse.T @ transverse / max(
            1, transverse.shape[0] - 1
        )
        transverse_values, transverse_vectors = np.linalg.eigh(transverse_covariance)
        transverse_axis = transverse_vectors[:, -1]
        transverse_projection = transverse @ transverse_axis
        width = float(
            np.percentile(transverse_projection, 95.0)
            - np.percentile(transverse_projection, 5.0)
        )
        width_profile.append(width if math.isfinite(width) and width > 0.0 else None)
        transverse_radius = float(np.percentile(
            np.linalg.norm(transverse, axis=1), 80.0
        ))
        radius_profile.append(
            transverse_radius
            if math.isfinite(transverse_radius) and transverse_radius > 0.0
            else None
        )

    radius_shape = _wrist_radius_shape(radius_positions, radius_profile)

    valid_widths = np.asarray(
        [width for width in width_profile if width is not None], dtype=np.float64
    )
    median_width = float(np.median(valid_widths)) if valid_widths.size else None
    width_cv = (
        float(np.std(valid_widths) / max(float(np.mean(valid_widths)), 1e-12))
        if valid_widths.size >= 2 else None
    )
    adjacent_jumps = [
        abs(float(width_profile[index + 1]) - float(width_profile[index]))
        / max(float(median_width or 0.0), 1e-12)
        for index in range(len(width_profile) - 1)
        if width_profile[index] is not None and width_profile[index + 1] is not None
    ]
    maximum_width_jump = max(adjacent_jumps) if adjacent_jumps else None

    maximum_run = 0
    current_run = 0
    for center in section_centers:
        current_run = current_run + 1 if center is not None else 0
        maximum_run = max(maximum_run, current_run)
    bin_width = axial_span / max(1, section_count)
    continuous_span = min(effective_length, maximum_run * bin_width)
    valid_section_count = sum(center is not None for center in section_centers)
    continuity_score = float(np.clip(
        continuous_span / max(effective_length, 1e-12)
        * valid_section_count / max(1, section_count),
        0.0,
        1.0,
    ))

    valid_counts = np.asarray(
        [count for count, center in zip(section_counts, section_centers) if center is not None],
        dtype=np.float64,
    )
    point_count_cv = (
        float(np.std(valid_counts) / max(float(np.mean(valid_counts)), 1e-12))
        if valid_counts.size >= 2 else 1.0
    )
    center_residuals = []
    local_axis_angles = []
    valid_centers = [center for center in section_centers if center is not None]
    for center in valid_centers:
        delta = center - wrist_point
        center_residuals.append(float(np.linalg.norm(delta - np.dot(delta, axis) * axis)))
    for first, second in zip(valid_centers, valid_centers[1:]):
        local_axis = second - first
        local_norm = float(np.linalg.norm(local_axis))
        if local_norm <= 1e-9:
            continue
        cosine = abs(float(np.dot(local_axis / local_norm, axis)))
        local_axis_angles.append(float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0)))))
    center_rms = (
        float(math.sqrt(np.mean(np.square(center_residuals))))
        if center_residuals else math.inf
    )
    section_stability = float(np.clip(
        math.exp(-point_count_cv)
        * math.exp(-float(width_cv or 0.0))
        * math.exp(-min(center_rms, 0.030) / 0.010),
        0.0,
        1.0,
    ))

    linearity = planarity = scattering = None
    if eigenvalues is not None and eigenvalues.size == 3 and eigenvalues[-1] > 1e-12:
        linearity = float((eigenvalues[-1] - eigenvalues[-2]) / eigenvalues[-1])
        planarity = float((eigenvalues[-2] - eigenvalues[-3]) / eigenvalues[-1])
        scattering = float(eigenvalues[-3] / eigenvalues[-1])
    local_geometry = WristGeometryLocalGeometry(
        linearity=linearity,
        planarity=planarity,
        scattering=scattering,
        local_axis_p95_deg=(
            float(np.percentile(local_axis_angles, 95.0))
            if local_axis_angles else None
        ),
    )

    directions = relative / np.maximum(
        np.linalg.norm(relative, axis=1, keepdims=True), 1e-12
    )
    mean_direction = np.mean(directions, axis=0)
    mean_direction /= max(float(np.linalg.norm(mean_direction)), 1e-12)
    angular_spread = np.degrees(np.arccos(np.clip(directions @ mean_direction, -1.0, 1.0)))
    angular_p95 = float(np.percentile(angular_spread, 95.0))
    sibling_complexity = float(np.clip((sibling_count - 1) / 3.0, 0.0, 1.0))
    branching_complexity = float(np.clip(
        0.6 * sibling_complexity + 0.4 * angular_p95 / 50.0, 0.0, 1.0
    ))

    near_threshold, far_threshold = np.percentile(axial, (10.0, 90.0))
    near_points = relative[axial <= near_threshold]
    far_points = relative[axial >= far_threshold]
    descriptor = WristGeometryBranchDescriptor(
        axis=tuple(float(value) for value in axis),
        centroid_m=tuple(float(value) for value in relative_centroid),
        near_endpoint_m=tuple(float(value) for value in np.median(near_points, axis=0)),
        far_endpoint_m=tuple(float(value) for value in np.median(far_points, axis=0)),
        effective_length_m=effective_length,
        median_width_m=median_width,
    )
    topology = _extract_wrist_topology_features(
        parent_points,
        parent_pixels,
        wrist_point,
        axis,
        effective_length,
        graph_stride,
    )
    depth_surface = _extract_wrist_depth_surface_features(
        branch_points,
        branch_pixels,
        parent_points,
        parent_pixels,
        wrist_point,
        axis,
        graph_stride,
    )
    return WristGeometryBranchFeatures(
        effective_length_m=effective_length,
        continuous_span_m=continuous_span,
        width_profile_m=tuple(width_profile),
        width_cv=width_cv,
        max_width_jump=maximum_width_jump,
        section_stability=section_stability,
        continuity_score=continuity_score,
        branching_complexity=branching_complexity,
        local_geometry=local_geometry,
        descriptor=descriptor,
        radius_shape=radius_shape,
        topology=topology,
        depth_surface=depth_surface,
    )


def _refine_wrist_branch_features(
    features: WristGeometryBranchFeatures,
    sections: tuple[WristGeometrySection, ...],
    axis: ArmDirection | None,
    axis_length_m: float | None,
    rms_m: float | None,
) -> WristGeometryBranchFeatures:
    """Add RANSAC continuity to shadow features after V2.1 has fitted a line."""
    inlier_sections = sorted(
        (section for section in sections if section.inlier),
        key=lambda section: section.section_index,
    )
    all_positions = sorted(section.axial_position_m for section in sections)
    spacing = (
        float(np.median(np.diff(all_positions)))
        if len(all_positions) >= 2 else 0.0
    )
    best_span = 0.0
    run: list[WristGeometrySection] = []
    for section in inlier_sections:
        if run and section.section_index != run[-1].section_index + 1:
            best_span = max(
                best_span,
                run[-1].axial_position_m - run[0].axial_position_m + spacing,
            )
            run = []
        run.append(section)
    if run:
        best_span = max(
            best_span,
            run[-1].axial_position_m - run[0].axial_position_m + spacing,
        )
    effective_length = features.effective_length_m
    if axis_length_m is not None and axis_length_m > 0.0:
        effective_length = min(effective_length, axis_length_m)
    continuous_span = min(effective_length, best_span)
    valid_count = sum(section.valid for section in sections)
    inlier_ratio = len(inlier_sections) / max(1, valid_count)
    continuity_score = float(np.clip(
        continuous_span / max(effective_length, 1e-12) * inlier_ratio,
        0.0,
        1.0,
    ))
    stability = features.section_stability
    if rms_m is not None:
        stability *= math.exp(-min(max(rms_m, 0.0), 0.030) / 0.006)
    stability *= math.sqrt(max(0.0, inlier_ratio))
    descriptor = replace(
        features.descriptor,
        axis=axis.vector if axis is not None else features.descriptor.axis,
        effective_length_m=effective_length,
    )
    return replace(
        features,
        effective_length_m=effective_length,
        continuous_span_m=continuous_span,
        continuity_score=continuity_score,
        section_stability=float(np.clip(stability, 0.0, 1.0)),
        descriptor=descriptor,
    )


def estimate_forearm_axis_wrist_geometry(
    landmarks,
    arm_mask: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
    wrist_search_radius_px: int = 18,
    dead_zone_m: float = 0.025,
    maximum_search_radius_m: float = 0.25,
    maximum_graph_points: int = 3000,
    previous_active_descriptor: WristGeometryBranchDescriptor | None = None,
) -> WristGeometryAxisEstimate:
    """Shadow-estimate a forearm axis using only landmark 0 and depth geometry.

    The estimator deliberately does not call ``_landmark_pixels`` because that
    helper iterates over every MediaPipe landmark.  Landmark 0 supplies only a
    wrist seed; every direction used below comes from the wrist-connected 3D
    cloud.
    """

    def rejected(
        reason: str,
        branches: tuple[WristGeometryBranch, ...] = (),
        parent_topology_shadow: dict | None = None,
        parent_split_shadow: dict | None = None,
    ) -> WristGeometryAxisEstimate:
        return WristGeometryAxisEstimate(
            direction=None,
            quality="V2_REJECTED",
            branch_count=len(branches),
            selected_branch=None,
            axis_length_m=None,
            rms_m=None,
            reason=reason,
            branches=branches,
            parent_topology_shadow=parent_topology_shadow,
            parent_split_shadow=parent_split_shadow,
        )

    if landmarks is None or intrinsics is None:
        return rejected("INVALID_INPUT")
    if arm_mask.shape != depth_m.shape or not np.any(arm_mask > 0):
        return rejected("MASK_EMPTY")
    try:
        wrist_landmark = landmarks[0]
        wrist_x = float(wrist_landmark.x)
        wrist_y = float(wrist_landmark.y)
    except (IndexError, TypeError, AttributeError, ValueError):
        return rejected("WRIST_LANDMARK_INVALID")
    if not math.isfinite(wrist_x + wrist_y):
        return rejected("WRIST_LANDMARK_INVALID")

    height, width = depth_m.shape
    wrist_pixel = np.asarray((
        np.clip(wrist_x, 0.0, 1.0) * (width - 1),
        np.clip(wrist_y, 0.0, 1.0) * (height - 1),
    ), dtype=np.float64)
    valid_input = (
        (arm_mask > 0)
        & np.isfinite(depth_m)
        & (depth_m >= 0.10)
        & (depth_m <= 2.0)
    )
    valid_input_count = int(np.count_nonzero(valid_input))
    graph_stride = max(
        1,
        int(math.ceil(math.sqrt(
            valid_input_count / max(1, maximum_graph_points)
        ))),
    )
    graph_mask = np.zeros_like(arm_mask, dtype=np.uint8)
    graph_mask[::graph_stride, ::graph_stride] = np.uint8(
        valid_input[::graph_stride, ::graph_stride]
    ) * 255
    points, pixels = masked_depth_to_points(
        graph_mask, depth_m, intrinsics, mirrored=mirrored, maximum_points=0
    )
    if points.shape[0] < 80:
        return rejected("CONNECTED_CLOUD_TOO_SMALL")

    pixel_delta = pixels - wrist_pixel
    pixel_distance_sq = np.sum(np.square(pixel_delta), axis=1)
    nearby = np.flatnonzero(
        pixel_distance_sq <= float(max(1, wrist_search_radius_px) ** 2)
    )
    if nearby.size < 5:
        return rejected("WRIST_SEED_UNSUPPORTED")
    seed_index = int(nearby[np.argmin(pixel_distance_sq[nearby])])
    wrist_point = points[seed_index]
    if not np.all(np.isfinite(wrist_point)):
        return rejected("WRIST_DEPTH_INVALID")

    index_map = np.full((height, width), -1, dtype=np.int32)
    pixel_columns = pixels[:, 0].astype(np.int32)
    pixel_rows = pixels[:, 1].astype(np.int32)
    index_map[pixel_rows, pixel_columns] = np.arange(points.shape[0], dtype=np.int32)
    neighbor_offsets = (
        (-graph_stride, -graph_stride), (0, -graph_stride),
        (graph_stride, -graph_stride), (-graph_stride, 0),
        (graph_stride, 0), (-graph_stride, graph_stride),
        (0, graph_stride), (graph_stride, graph_stride),
    )

    def connected_neighbors(point_index: int):
        column = int(pixel_columns[point_index])
        row = int(pixel_rows[point_index])
        point = points[point_index]
        for dx, dy in neighbor_offsets:
            neighbor_column = column + dx
            neighbor_row = row + dy
            if not (0 <= neighbor_column < width and 0 <= neighbor_row < height):
                continue
            neighbor_index = int(index_map[neighbor_row, neighbor_column])
            if neighbor_index < 0:
                continue
            neighbor = points[neighbor_index]
            edge_threshold = max(
                0.006, 0.012 * min(float(point[2]), float(neighbor[2]))
            )
            if float(np.linalg.norm(neighbor - point)) <= edge_threshold:
                yield neighbor_index

    connected = np.zeros(points.shape[0], dtype=bool)
    connected[seed_index] = True
    queue = deque([seed_index])
    while queue:
        point_index = queue.popleft()
        for neighbor_index in connected_neighbors(point_index):
            if connected[neighbor_index]:
                continue
            if float(np.linalg.norm(points[neighbor_index] - wrist_point)) > (
                maximum_search_radius_m
            ):
                continue
            connected[neighbor_index] = True
            queue.append(neighbor_index)
    if int(np.count_nonzero(connected)) < 80:
        return rejected("CONNECTED_CLOUD_TOO_SMALL")

    wrist_distance = np.linalg.norm(points - wrist_point, axis=1)
    branch_eligible = (
        connected
        & (wrist_distance >= dead_zone_m)
        & (wrist_distance <= maximum_search_radius_m)
    )
    component_visited = np.zeros(points.shape[0], dtype=bool)
    component_indices: list[tuple[int, int, np.ndarray]] = []
    parent_component_indices: dict[int, np.ndarray] = {}
    parent_component_id = 0
    for start_index in np.flatnonzero(branch_eligible):
        if component_visited[start_index]:
            continue
        component_visited[start_index] = True
        component_queue = deque([int(start_index)])
        component: list[int] = []
        while component_queue:
            point_index = component_queue.popleft()
            component.append(point_index)
            for neighbor_index in connected_neighbors(point_index):
                if (
                    branch_eligible[neighbor_index]
                    and not component_visited[neighbor_index]
                ):
                    component_visited[neighbor_index] = True
                    component_queue.append(neighbor_index)
        if len(component) < 40:
            continue
        component_array = np.asarray(component, dtype=np.int64)
        parent_component_indices[parent_component_id] = component_array
        component_distance = wrist_distance[component_array]
        far_threshold = max(
            dead_zone_m + 0.020,
            float(np.percentile(component_distance, 65.0)),
        )
        far_indices = component_array[component_distance >= far_threshold]
        if far_indices.size < 10:
            component_indices.append((parent_component_id, 1, component_array))
            parent_component_id += 1
            continue
        far_indices = far_indices[
            np.argsort(wrist_distance[far_indices])[::-1]
        ]
        direction_sums: list[np.ndarray] = []
        cluster_cosine = math.cos(math.radians(50.0))
        for point_index in far_indices:
            direction = points[point_index] - wrist_point
            direction /= max(float(np.linalg.norm(direction)), 1e-12)
            if not direction_sums:
                direction_sums.append(direction.copy())
                continue
            centers = [
                vector / max(float(np.linalg.norm(vector)), 1e-12)
                for vector in direction_sums
            ]
            similarities = [float(np.dot(direction, center)) for center in centers]
            best_cluster = int(np.argmax(similarities))
            if similarities[best_cluster] >= cluster_cosine:
                direction_sums[best_cluster] += direction
            elif len(direction_sums) < 6:
                direction_sums.append(direction.copy())
        centers = np.asarray([
            vector / max(float(np.linalg.norm(vector)), 1e-12)
            for vector in direction_sums
        ])
        component_vectors = points[component_array] - wrist_point
        component_vectors /= np.maximum(
            np.linalg.norm(component_vectors, axis=1, keepdims=True), 1e-12
        )
        assignments = np.argmax(component_vectors @ centers.T, axis=1)
        directional_components = []
        for cluster_index in range(centers.shape[0]):
            directional_component = component_array[assignments == cluster_index]
            if directional_component.size >= 40:
                directional_components.append(directional_component)
        sibling_count = max(1, len(directional_components))
        component_indices.extend(
            (parent_component_id, sibling_count, directional_component)
            for directional_component in directional_components
        )
        parent_component_id += 1
    if not component_indices:
        return rejected("NO_BRANCH_AFTER_WRIST_CUT")

    branches: list[WristGeometryBranch] = []
    branch_indices_by_id: dict[int, np.ndarray] = {}
    for branch_id, (
        branch_parent_component_id,
        branch_sibling_count,
        indices,
    ) in enumerate(component_indices):
        branch_indices_by_id[branch_id] = indices
        branch_points = points[indices]
        radial_distance = wrist_distance[indices]
        span_m = float(np.max(radial_distance) - np.min(radial_distance))
        branch_origin = np.mean(branch_points, axis=0)
        centered = branch_points - branch_origin
        covariance = centered.T @ centered / max(1, branch_points.shape[0] - 1)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        feature_axis = None
        if np.all(np.isfinite(eigenvalues)) and eigenvalues[-1] > 1e-10:
            feature_axis = eigenvectors[:, -1]
            feature_farthest = branch_points[int(np.argmax(radial_distance))]
            if float(np.dot(feature_axis, feature_farthest - wrist_point)) < 0.0:
                feature_axis = -feature_axis
        branch_features = _extract_wrist_branch_features(
            branch_points,
            wrist_point,
            feature_axis,
            eigenvalues if feature_axis is not None else None,
            branch_sibling_count,
            branch_pixels=pixels[indices],
            parent_points=points[parent_component_indices[branch_parent_component_id]],
            parent_pixels=pixels[parent_component_indices[branch_parent_component_id]],
            graph_stride=graph_stride,
        )

        def branch_result(
            reason: str,
            *,
            pca_direction: ArmDirection | None = None,
            pca_linearity: float | None = None,
            sections: tuple[WristGeometrySection, ...] = (),
            inlier_indices: np.ndarray | None = None,
            axis: ArmDirection | None = None,
            axis_length_m: float | None = None,
            rms_m: float | None = None,
            valid: bool = False,
            features: WristGeometryBranchFeatures | None = None,
            local_recovery_shadow: dict | None = None,
        ) -> WristGeometryBranch:
            inlier_count = int(inlier_indices.size) if inlier_indices is not None else 0
            valid_count = sum(section.valid for section in sections)
            return WristGeometryBranch(
                branch_id=branch_id,
                point_count=int(indices.size),
                span_m=span_m,
                pca_direction=pca_direction,
                pca_linearity=pca_linearity,
                valid_section_count=valid_count,
                inlier_section_count=inlier_count,
                inlier_ratio=float(inlier_count / max(1, valid_count)),
                axis=axis,
                axis_length_m=axis_length_m,
                rms_m=rms_m,
                valid=valid,
                reason=reason,
                sections=sections,
                parent_component_id=branch_parent_component_id,
                features=features if features is not None else branch_features,
                local_recovery_shadow=local_recovery_shadow,
            )

        if span_m < 0.040:
            branches.append(branch_result("BRANCH_TOO_SHORT"))
            continue
        if not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 1e-10:
            branches.append(branch_result("BRANCH_PCA_DEGENERATE"))
            continue
        pca_axis = feature_axis
        farthest = branch_points[int(np.argmax(radial_distance))]
        pca_linearity = float(eigenvalues[-1] / max(eigenvalues[-2], 1e-12))
        pca_direction = ArmDirection(tuple(float(value) for value in pca_axis))
        if pca_linearity < 2.5:
            local_recovery_shadow = analyze_branch_local_recovery_shadow(
                branch_id,
                branch_points,
                pca_axis,
                wrist_point,
                original_reason="BRANCH_NOT_LINEAR",
                global_linearity=pca_linearity,
            )
            branches.append(branch_result(
                "BRANCH_NOT_LINEAR",
                pca_direction=pca_direction,
                pca_linearity=pca_linearity,
                local_recovery_shadow=local_recovery_shadow,
            ))
            continue

        axial = (branch_points - wrist_point) @ pca_axis
        positive = axial > 0.5 * dead_zone_m
        branch_points = branch_points[positive]
        axial = axial[positive]
        if axial.size < 40:
            branches.append(branch_result(
                "BRANCH_TOO_SHORT",
                pca_direction=pca_direction,
                pca_linearity=pca_linearity,
            ))
            continue
        lower, upper = np.percentile(axial, (2.0, 98.0))
        axial_span = float(upper - lower)
        if not math.isfinite(axial_span) or axial_span < 0.040:
            branches.append(branch_result(
                "BRANCH_TOO_SHORT",
                pca_direction=pca_direction,
                pca_linearity=pca_linearity,
            ))
            continue
        section_count = int(np.clip(math.ceil(axial_span / 0.020), 5, 12))
        edges = np.linspace(lower, upper, section_count + 1)
        minimum_section_points = max(10, int(branch_points.shape[0] / section_count * 0.15))
        sections_list: list[WristGeometrySection] = []
        for section_index in range(section_count):
            selected = (axial >= edges[section_index]) & (
                axial <= edges[section_index + 1]
                if section_index == section_count - 1
                else axial < edges[section_index + 1]
            )
            section_points = branch_points[selected]
            axial_position = float((edges[section_index] + edges[section_index + 1]) * 0.5)
            if section_points.shape[0] < minimum_section_points:
                sections_list.append(WristGeometrySection(
                    section_index=section_index,
                    axial_position_m=axial_position,
                    center_m=None,
                    point_count=int(section_points.shape[0]),
                    depth_spread_m=None,
                    valid=False,
                ))
                continue
            center = np.median(section_points, axis=0)
            depth_deviation = np.abs(
                section_points[:, 2] - np.median(section_points[:, 2])
            )
            sections_list.append(WristGeometrySection(
                section_index=section_index,
                axial_position_m=axial_position,
                center_m=tuple(float(value) for value in center),
                point_count=int(section_points.shape[0]),
                depth_spread_m=float(1.4826 * np.median(depth_deviation)),
                valid=True,
            ))
        sections = tuple(sections_list)
        valid_sections = [section for section in sections if section.valid]
        if len(valid_sections) < 4:
            branches.append(branch_result(
                "TOO_FEW_SECTIONS",
                pca_direction=pca_direction,
                pca_linearity=pca_linearity,
                sections=sections,
            ))
            continue
        _origin, _direction, inlier_indices, ransac_reason = (
            fit_forearm_centerline_ransac(
                list(sections), distance_threshold_m=0.006,
                minimum_inlier_sections=4,
            )
        )
        if ransac_reason != "NONE":
            branches.append(branch_result(
                ransac_reason,
                pca_direction=pca_direction,
                pca_linearity=pca_linearity,
                sections=sections,
                inlier_indices=inlier_indices,
            ))
            continue
        inlier_set = {int(value) for value in inlier_indices}
        sections = tuple(
            replace(
                section,
                inlier=section.valid and section.section_index in inlier_set,
            )
            for section in sections
        )
        inlier_sections = [section for section in sections if section.inlier]
        inlier_centers = np.asarray(
            [section.center_m for section in inlier_sections], dtype=np.float64
        )
        line_origin = np.mean(inlier_centers, axis=0)
        line_centered = inlier_centers - line_origin
        line_covariance = line_centered.T @ line_centered / max(
            1, inlier_centers.shape[0] - 1
        )
        line_eigenvalues, line_eigenvectors = np.linalg.eigh(line_covariance)
        if line_eigenvalues[-1] <= 1e-10:
            branches.append(branch_result(
                "RANSAC_REFINE_DEGENERATE",
                pca_direction=pca_direction,
                pca_linearity=pca_linearity,
                sections=sections,
                inlier_indices=inlier_indices,
            ))
            continue
        axis_vector = line_eigenvectors[:, -1]
        if float(np.dot(axis_vector, farthest - wrist_point)) < 0.0:
            axis_vector = -axis_vector
        projections = line_centered @ axis_vector
        axis_length_m = float(np.max(projections) - np.min(projections))
        residuals = np.linalg.norm(
            line_centered - np.outer(projections, axis_vector), axis=1
        )
        rms_m = float(math.sqrt(np.mean(np.square(residuals))))
        inlier_ratio = float(len(inlier_sections) / max(1, len(valid_sections)))
        ordered_inliers = sorted(inlier_set)
        maximum_run = 0
        current_run = 0
        previous = None
        for section_index in ordered_inliers:
            current_run = current_run + 1 if previous is not None and section_index == previous + 1 else 1
            maximum_run = max(maximum_run, current_run)
            previous = section_index
        reason = "NONE"
        if inlier_ratio < 0.60:
            reason = "INLIER_RATIO_LOW"
        elif maximum_run < 4:
            reason = "INLIER_CONTINUITY_LOW"
        elif axis_length_m < 0.060:
            reason = "TOO_SHORT"
        elif rms_m > 0.006:
            reason = "RMS_TOO_HIGH"
        axis = ArmDirection(tuple(float(value) for value in axis_vector))
        refined_features = _refine_wrist_branch_features(
            branch_features, sections, axis, axis_length_m, rms_m
        )
        branches.append(branch_result(
            reason,
            pca_direction=pca_direction,
            pca_linearity=pca_linearity,
            sections=sections,
            inlier_indices=inlier_indices,
            axis=axis,
            axis_length_m=axis_length_m,
            rms_m=rms_m,
            valid=reason == "NONE",
            features=refined_features,
        ))

    branch_tuple = tuple(branches)
    parent_topology_shadow = analyze_branch_parent_topology_shadow(
        wrist_point,
        points,
        parent_component_indices,
        branch_indices_by_id,
        branch_tuple,
        previous_active_descriptor=previous_active_descriptor,
    )
    parent_split_shadow = analyze_parent_component_split_shadow(
        wrist_point,
        points,
        pixels,
        parent_component_indices,
        parent_topology_shadow,
        previous_active_descriptor=previous_active_descriptor,
        graph_stride=graph_stride,
    )
    valid_branches = [branch for branch in branches if branch.valid]
    if not valid_branches:
        failure_reason = (
            branches[0].reason if len(branches) == 1 else "NO_VALID_BRANCH"
        )
        return rejected(
            failure_reason,
            branch_tuple,
            parent_topology_shadow=parent_topology_shadow,
            parent_split_shadow=parent_split_shadow,
        )
    valid_branches.sort(key=lambda branch: (
        -(branch.axis_length_m or 0.0),
        branch.rms_m if branch.rms_m is not None else math.inf,
    ))
    selected = valid_branches[0]
    if len(valid_branches) > 1:
        second = valid_branches[1]
        if (second.axis_length_m or 0.0) >= 0.85 * (selected.axis_length_m or 0.0):
            return rejected(
                "AMBIGUOUS_BRANCH",
                branch_tuple,
                parent_topology_shadow=parent_topology_shadow,
                parent_split_shadow=parent_split_shadow,
            )
    quality = (
        "V2_STRONG"
        if (selected.axis_length_m or 0.0) >= 0.10
        else "V2_DEGRADED"
    )
    return WristGeometryAxisEstimate(
        direction=selected.axis,
        quality=quality,
        branch_count=len(branches),
        selected_branch=selected.branch_id,
        axis_length_m=selected.axis_length_m,
        rms_m=selected.rms_m,
        reason="NONE",
        branches=branch_tuple,
        parent_topology_shadow=parent_topology_shadow,
        parent_split_shadow=parent_split_shadow,
    )


def _wrist_branch_descriptor_match_cost(
    previous: WristGeometryBranchDescriptor,
    current: WristGeometryBranchDescriptor,
) -> float | None:
    """Return an explainable gated cost for two wrist-relative branches."""
    return _wrist_branch_descriptor_match_diagnostic(previous, current)[0]


def _wrist_branch_descriptor_match_diagnostic(
    previous: WristGeometryBranchDescriptor,
    current: WristGeometryBranchDescriptor,
) -> tuple[float | None, str, dict]:
    """Mirror existing match gates while exposing why a pair was rejected."""
    if previous.axis is None or current.axis is None:
        return None, "MATCH_AXIS_MISSING", {}
    previous_axis = np.asarray(previous.axis, dtype=np.float64)
    current_axis = np.asarray(current.axis, dtype=np.float64)
    cosine = abs(float(np.dot(previous_axis, current_axis))) / max(
        float(np.linalg.norm(previous_axis) * np.linalg.norm(current_axis)), 1e-12
    )
    axis_angle_deg = float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))
    centroid_distance = float(np.linalg.norm(
        np.asarray(previous.centroid_m) - np.asarray(current.centroid_m)
    ))
    far_endpoint_distance = float(np.linalg.norm(
        np.asarray(previous.far_endpoint_m) - np.asarray(current.far_endpoint_m)
    ))
    length_delta = abs(previous.effective_length_m - current.effective_length_m) / max(
        previous.effective_length_m, current.effective_length_m, 1e-12
    )
    width_delta = 0.0
    if previous.median_width_m is not None and current.median_width_m is not None:
        width_delta = abs(previous.median_width_m - current.median_width_m) / max(
            previous.median_width_m, current.median_width_m, 1e-12
        )
    details = {
        "axis_angle_deg": axis_angle_deg,
        "centroid_distance_m": centroid_distance,
        "far_endpoint_distance_m": far_endpoint_distance,
        "length_relative_delta": length_delta,
        "width_relative_delta": width_delta,
    }
    if axis_angle_deg > 35.0:
        return None, "MATCH_AXIS_GATE", details
    if centroid_distance > 0.10:
        return None, "MATCH_CENTROID_GATE", details
    if far_endpoint_distance > 0.12:
        return None, "MATCH_ENDPOINT_GATE", details
    if length_delta > 0.70:
        return None, "MATCH_LENGTH_GATE", details
    cost = float(
        0.40 * axis_angle_deg / 35.0
        + 0.25 * centroid_distance / 0.10
        + 0.20 * far_endpoint_distance / 0.12
        + 0.10 * length_delta / 0.70
        + 0.05 * min(width_delta, 1.0)
    )
    return cost, ("MATCH_OK" if cost <= 1.0 else "MATCH_TOTAL_COST_GATE"), details


def _blend_wrist_branch_descriptor(
    previous: WristGeometryBranchDescriptor,
    current: WristGeometryBranchDescriptor,
    current_weight: float = 0.35,
) -> WristGeometryBranchDescriptor:
    """Smooth a matched descriptor while preserving its unoriented axis."""
    weight = float(np.clip(current_weight, 0.0, 1.0))

    def blend_tuple(first, second):
        return tuple(float(value) for value in (
            (1.0 - weight) * np.asarray(first, dtype=np.float64)
            + weight * np.asarray(second, dtype=np.float64)
        ))

    blended_axis = current.axis
    if previous.axis is not None and current.axis is not None:
        previous_axis = np.asarray(previous.axis, dtype=np.float64)
        current_axis = np.asarray(current.axis, dtype=np.float64)
        if float(np.dot(previous_axis, current_axis)) < 0.0:
            current_axis = -current_axis
        axis = (1.0 - weight) * previous_axis + weight * current_axis
        axis /= max(float(np.linalg.norm(axis)), 1e-12)
        blended_axis = tuple(float(value) for value in axis)
    median_width = current.median_width_m
    if previous.median_width_m is not None and current.median_width_m is not None:
        median_width = (
            (1.0 - weight) * previous.median_width_m
            + weight * current.median_width_m
        )
    return WristGeometryBranchDescriptor(
        axis=blended_axis,
        centroid_m=blend_tuple(previous.centroid_m, current.centroid_m),
        near_endpoint_m=blend_tuple(previous.near_endpoint_m, current.near_endpoint_m),
        far_endpoint_m=blend_tuple(previous.far_endpoint_m, current.far_endpoint_m),
        effective_length_m=(
            (1.0 - weight) * previous.effective_length_m
            + weight * current.effective_length_m
        ),
        median_width_m=median_width,
    )


def _ema_scalar(
    previous: float | None,
    variance: float,
    value: float | None,
    alpha: float,
) -> tuple[float | None, float]:
    if value is None or not math.isfinite(value):
        return previous, variance
    if previous is None or not math.isfinite(previous):
        return float(value), 0.0
    delta = float(value - previous)
    updated = previous + alpha * delta
    updated_variance = (1.0 - alpha) * (
        variance + alpha * delta * delta
    )
    return float(updated), float(max(0.0, updated_variance))


def _resample_width_profile(
    profile: tuple[float | None, ...],
    output_count: int = 8,
) -> tuple[float | None, ...]:
    if not profile:
        return (None,) * output_count
    source_positions = np.linspace(0.0, 1.0, len(profile))
    valid_indices = [index for index, value in enumerate(profile) if value is not None]
    if not valid_indices:
        return (None,) * output_count
    valid_positions = source_positions[valid_indices]
    valid_values = np.asarray([profile[index] for index in valid_indices], dtype=np.float64)
    if len(valid_indices) == 1:
        return tuple(float(valid_values[0]) for _ in range(output_count))
    result = np.interp(
        np.linspace(0.0, 1.0, output_count), valid_positions, valid_values
    )
    return tuple(float(value) for value in result)


def _track_radial_progression(
    descriptor: WristGeometryBranchDescriptor,
) -> float:
    near_radius = float(np.linalg.norm(np.asarray(descriptor.near_endpoint_m)))
    far_radius = float(np.linalg.norm(np.asarray(descriptor.far_endpoint_m)))
    return float(np.clip(
        (far_radius - near_radius) / max(descriptor.effective_length_m, 1e-12),
        0.0,
        1.0,
    ))


def _update_track_feature_ema(
    track: WristGeometryBranchTrack,
    features: WristGeometryBranchFeatures,
    axis_delta_deg: float,
    timestamp_sec: float | None,
) -> None:
    ema = track.feature_ema
    if timestamp_sec is not None and ema.last_update_time is not None:
        delta_time = max(1e-3, timestamp_sec - ema.last_update_time)
    else:
        delta_time = 1.0 / 30.0
    alpha = float(np.clip(1.0 - math.exp(-delta_time / 0.8), 0.02, 0.50))
    ema.length_m, ema.length_variance = _ema_scalar(
        ema.length_m, ema.length_variance, features.effective_length_m, alpha
    )
    ema.continuity, ema.continuity_variance = _ema_scalar(
        ema.continuity, ema.continuity_variance, features.continuity_score, alpha
    )
    ema.width_cv, ema.width_cv_variance = _ema_scalar(
        ema.width_cv, ema.width_cv_variance, features.width_cv, alpha
    )
    ema.max_width_jump, ema.max_width_jump_variance = _ema_scalar(
        ema.max_width_jump,
        ema.max_width_jump_variance,
        features.max_width_jump,
        alpha,
    )
    ema.section_stability, ema.section_stability_variance = _ema_scalar(
        ema.section_stability,
        ema.section_stability_variance,
        features.section_stability,
        alpha,
    )
    ema.branching_complexity, ema.branching_complexity_variance = _ema_scalar(
        ema.branching_complexity,
        ema.branching_complexity_variance,
        features.branching_complexity,
        alpha,
    )
    ema.axis_delta_deg, ema.axis_delta_variance = _ema_scalar(
        ema.axis_delta_deg, ema.axis_delta_variance, axis_delta_deg, alpha
    )
    ema.radial_progression, ema.radial_progression_variance = _ema_scalar(
        ema.radial_progression,
        ema.radial_progression_variance,
        _track_radial_progression(features.descriptor),
        alpha,
    )
    radius_shape = features.radius_shape
    ema.radius_gradient, ema.radius_gradient_variance = _ema_scalar(
        ema.radius_gradient,
        ema.radius_gradient_variance,
        radius_shape.gradient,
        alpha,
    )
    ema.radius_smoothness, ema.radius_smoothness_variance = _ema_scalar(
        ema.radius_smoothness,
        ema.radius_smoothness_variance,
        radius_shape.smoothness,
        alpha,
    )
    ema.radius_jump, ema.radius_jump_variance = _ema_scalar(
        ema.radius_jump,
        ema.radius_jump_variance,
        radius_shape.jump,
        alpha,
    )
    ema.radius_valid_ratio, ema.radius_valid_ratio_variance = _ema_scalar(
        ema.radius_valid_ratio,
        ema.radius_valid_ratio_variance,
        radius_shape.valid_ratio,
        alpha,
    )
    topology = features.topology
    ema.distal_branch_count, ema.distal_branch_count_variance = _ema_scalar(
        ema.distal_branch_count,
        ema.distal_branch_count_variance,
        float(topology.distal_branch_count)
        if topology.distal_branch_count is not None else None,
        alpha,
    )
    (
        ema.distal_angular_spread_deg,
        ema.distal_angular_spread_variance,
    ) = _ema_scalar(
        ema.distal_angular_spread_deg,
        ema.distal_angular_spread_variance,
        topology.distal_angular_spread_deg,
        alpha,
    )
    (
        ema.far_shell_component_count,
        ema.far_shell_component_count_variance,
    ) = _ema_scalar(
        ema.far_shell_component_count,
        ema.far_shell_component_count_variance,
        float(topology.far_shell_component_count)
        if topology.far_shell_component_count is not None else None,
        alpha,
    )
    ema.distal_valid_ratio, ema.distal_valid_ratio_variance = _ema_scalar(
        ema.distal_valid_ratio,
        ema.distal_valid_ratio_variance,
        1.0 if topology.distal_valid else 0.0,
        alpha,
    )
    (
        ema.wrist_bottleneck_radius_m,
        ema.wrist_bottleneck_radius_variance,
    ) = _ema_scalar(
        ema.wrist_bottleneck_radius_m,
        ema.wrist_bottleneck_radius_variance,
        topology.wrist_bottleneck_radius_m,
        alpha,
    )
    ema.growth_60mm, ema.growth_60mm_variance = _ema_scalar(
        ema.growth_60mm,
        ema.growth_60mm_variance,
        topology.growth_60mm,
        alpha,
    )
    ema.max_local_growth, ema.max_local_growth_variance = _ema_scalar(
        ema.max_local_growth,
        ema.max_local_growth_variance,
        topology.max_local_growth,
        alpha,
    )
    (
        ema.growth_concentration,
        ema.growth_concentration_variance,
    ) = _ema_scalar(
        ema.growth_concentration,
        ema.growth_concentration_variance,
        topology.growth_concentration,
        alpha,
    )
    ema.bottleneck_ratio, ema.bottleneck_ratio_variance = _ema_scalar(
        ema.bottleneck_ratio,
        ema.bottleneck_ratio_variance,
        topology.bottleneck_ratio,
        alpha,
    )
    (
        ema.bottleneck_valid_ratio,
        ema.bottleneck_valid_ratio_variance,
    ) = _ema_scalar(
        ema.bottleneck_valid_ratio,
        ema.bottleneck_valid_ratio_variance,
        1.0 if topology.bottleneck_valid else 0.0,
        alpha,
    )
    surface = features.depth_surface
    (
        ema.surface_transverse_curvature,
        ema.surface_transverse_curvature_variance,
    ) = _ema_scalar(
        ema.surface_transverse_curvature,
        ema.surface_transverse_curvature_variance,
        surface.transverse_curvature,
        alpha,
    )
    ema.surface_axial_curvature, ema.surface_axial_curvature_variance = _ema_scalar(
        ema.surface_axial_curvature,
        ema.surface_axial_curvature_variance,
        surface.axial_curvature,
        alpha,
    )
    (
        ema.surface_curvature_anisotropy,
        ema.surface_curvature_anisotropy_variance,
    ) = _ema_scalar(
        ema.surface_curvature_anisotropy,
        ema.surface_curvature_anisotropy_variance,
        surface.curvature_anisotropy,
        alpha,
    )
    (
        ema.surface_radius_estimate_m,
        ema.surface_radius_estimate_variance,
    ) = _ema_scalar(
        ema.surface_radius_estimate_m,
        ema.surface_radius_estimate_variance,
        surface.radius_estimate_m,
        alpha,
    )
    (
        ema.surface_normal_axial_mad_deg,
        ema.surface_normal_axial_mad_variance,
    ) = _ema_scalar(
        ema.surface_normal_axial_mad_deg,
        ema.surface_normal_axial_mad_variance,
        surface.normal_axial_mad_deg,
        alpha,
    )
    ema.surface_normal_jump_deg, ema.surface_normal_jump_variance = _ema_scalar(
        ema.surface_normal_jump_deg,
        ema.surface_normal_jump_variance,
        surface.normal_jump_deg,
        alpha,
    )
    (
        ema.surface_object_boundary_ratio,
        ema.surface_object_boundary_ratio_variance,
    ) = _ema_scalar(
        ema.surface_object_boundary_ratio,
        ema.surface_object_boundary_ratio_variance,
        surface.object_boundary_ratio,
        alpha,
    )
    (
        ema.surface_internal_cut_ratio,
        ema.surface_internal_cut_ratio_variance,
    ) = _ema_scalar(
        ema.surface_internal_cut_ratio,
        ema.surface_internal_cut_ratio_variance,
        surface.internal_cut_boundary_ratio,
        alpha,
    )
    ema.surface_curvature_score, ema.surface_curvature_score_variance = _ema_scalar(
        ema.surface_curvature_score,
        ema.surface_curvature_score_variance,
        surface.curvature_score if surface.curvature_valid else None,
        alpha,
    )
    ema.surface_normal_score, ema.surface_normal_score_variance = _ema_scalar(
        ema.surface_normal_score,
        ema.surface_normal_score_variance,
        surface.normal_score if surface.normal_valid else None,
        alpha,
    )
    ema.surface_boundary_score, ema.surface_boundary_score_variance = _ema_scalar(
        ema.surface_boundary_score,
        ema.surface_boundary_score_variance,
        surface.boundary_score if surface.boundary_valid else None,
        alpha,
    )
    ema.depth_surface_score_raw, ema.depth_surface_score_variance = _ema_scalar(
        ema.depth_surface_score_raw,
        ema.depth_surface_score_variance,
        surface.score_raw,
        alpha,
    )
    ema.depth_surface_evidence, ema.depth_surface_evidence_variance = _ema_scalar(
        ema.depth_surface_evidence,
        ema.depth_surface_evidence_variance,
        surface.evidence,
        alpha,
    )
    track.depth_surface_score_raw = surface.score_raw
    current_width_profile = _resample_width_profile(features.width_profile_m)
    previous_profile = list(ema.width_profile_m)
    previous_variance = list(ema.width_profile_variance)
    for index, width in enumerate(current_width_profile):
        previous_profile[index], previous_variance[index] = _ema_scalar(
            previous_profile[index], previous_variance[index], width, alpha
        )
    ema.width_profile_m = tuple(previous_profile)
    ema.width_profile_variance = tuple(previous_variance)
    current_radius_profile = radius_shape.profile_m
    previous_radius_profile = list(ema.radius_profile_m)
    previous_radius_variance = list(ema.radius_profile_variance)
    for index, radius in enumerate(current_radius_profile):
        previous_radius_profile[index], previous_radius_variance[index] = (
            _ema_scalar(
                previous_radius_profile[index],
                previous_radius_variance[index],
                radius,
                alpha,
            )
        )
    ema.radius_profile_m = tuple(previous_radius_profile)
    ema.radius_profile_variance = tuple(previous_radius_variance)
    ema.update_count += 1
    if timestamp_sec is not None:
        ema.last_update_time = timestamp_sec


def _smoothstep(lower: float, upper: float, value: float | None) -> float:
    if value is None or upper <= lower:
        return 0.0
    normalized = float(np.clip((value - lower) / (upper - lower), 0.0, 1.0))
    return normalized * normalized * (3.0 - 2.0 * normalized)


def _radius_gradient_score(gradient: float | None) -> float:
    """Allow gentle taper while rejecting abrupt expansion or contraction."""
    if gradient is None or not math.isfinite(gradient):
        return 0.5
    if -0.15 <= gradient <= 0.60:
        return 1.0
    if gradient < -0.15:
        return _smoothstep(-0.60, -0.15, gradient)
    return 1.0 - _smoothstep(0.60, 1.20, gradient)


def _bounded_plateau_score(
    value: float | None,
    minimum: float,
    lower: float,
    upper: float,
    maximum: float,
) -> float:
    if value is None or not math.isfinite(value):
        return 0.5
    if lower <= value <= upper:
        return 1.0
    if value < lower:
        return _smoothstep(minimum, lower, value)
    return 1.0 - _smoothstep(upper, maximum, value)


def _update_track_topology_score(track: WristGeometryBranchTrack) -> None:
    """Compute topology-only and combined shadow scores without selection."""
    ema = track.feature_ema
    distal_count_score = (
        math.exp(-((max(0.0, ema.distal_branch_count - 1.0) / 1.2) ** 2))
        if ema.distal_branch_count is not None else 0.5
    )
    distal_angle_score = (
        math.exp(-((ema.distal_angular_spread_deg / 25.0) ** 2))
        if ema.distal_angular_spread_deg is not None else 0.5
    )
    far_component_score = (
        math.exp(-((max(0.0, ema.far_shell_component_count - 1.0) / 1.2) ** 2))
        if ema.far_shell_component_count is not None else 0.5
    )
    fanout_score = (
        max(distal_count_score, 1e-12) ** 0.40
        * max(distal_angle_score, 1e-12) ** 0.35
        * max(far_component_score, 1e-12) ** 0.25
    )

    growth_score = _bounded_plateau_score(
        ema.growth_60mm, -0.60, -0.20, 0.80, 1.50
    )
    local_growth_score = (
        math.exp(-((ema.max_local_growth / 0.60) ** 2))
        if ema.max_local_growth is not None else 0.5
    )
    concentration_score = (
        1.0 - _smoothstep(0.50, 0.90, ema.growth_concentration)
        if ema.growth_concentration is not None else 0.5
    )
    ratio_score = (
        1.0 - _smoothstep(1.50, 2.50, ema.bottleneck_ratio)
        if ema.bottleneck_ratio is not None else 0.5
    )
    bottleneck_score = (
        max(growth_score, 1e-12) ** 0.25
        * max(local_growth_score, 1e-12) ** 0.30
        * max(concentration_score, 1e-12) ** 0.20
        * max(ratio_score, 1e-12) ** 0.25
    )
    topology_raw = 0.55 * fanout_score + 0.45 * bottleneck_score
    distal_evidence = float(np.clip(ema.distal_valid_ratio or 0.0, 0.0, 1.0))
    bottleneck_evidence = float(np.clip(
        ema.bottleneck_valid_ratio or 0.0, 0.0, 1.0
    ))
    coverage_evidence = 0.55 * distal_evidence + 0.45 * bottleneck_evidence
    topology_maturity = math.sqrt(
        _smoothstep(1.0, 8.0, float(ema.update_count)) * coverage_evidence
    )
    topology_effective = float(
        0.5 + topology_maturity * (topology_raw - 0.5)
    )
    track.topology_score_raw = float(np.clip(topology_raw, 0.0, 1.0))
    track.topology_maturity = float(np.clip(topology_maturity, 0.0, 1.0))
    track.topology_score = float(np.clip(topology_effective, 0.0, 1.0))
    track.forearm_score_topology = float(np.clip(
        0.75 * track.forearm_score_v2 + 0.25 * track.topology_score,
        0.0,
        1.0,
    ))
    track.topology_components = {
        "distal_branch_count": ema.distal_branch_count,
        "distal_angular_spread": ema.distal_angular_spread_deg,
        "far_shell_component_count": ema.far_shell_component_count,
        "distal_count_score": distal_count_score,
        "distal_angle_score": distal_angle_score,
        "far_component_score": far_component_score,
        "fanout_score": fanout_score,
        "wrist_bottleneck_radius": ema.wrist_bottleneck_radius_m,
        "growth_60mm": ema.growth_60mm,
        "max_local_growth": ema.max_local_growth,
        "growth_concentration": ema.growth_concentration,
        "bottleneck_ratio": ema.bottleneck_ratio,
        "growth_score": growth_score,
        "local_growth_score": local_growth_score,
        "concentration_score": concentration_score,
        "ratio_score": ratio_score,
        "bottleneck_score": bottleneck_score,
        "distal_evidence": distal_evidence,
        "bottleneck_evidence": bottleneck_evidence,
    }


def _update_track_depth_surface_score(track: WristGeometryBranchTrack) -> None:
    """Publish an independent surface score; never alter existing likelihoods."""
    ema = track.feature_ema
    score_ema = float(np.clip(
        ema.depth_surface_score_raw
        if ema.depth_surface_score_raw is not None else 0.5,
        0.0,
        1.0,
    ))
    evidence = float(np.clip(
        ema.depth_surface_evidence
        if ema.depth_surface_evidence is not None else 0.0,
        0.0,
        1.0,
    ))
    maturity = math.sqrt(
        _smoothstep(1.0, 8.0, float(ema.update_count)) * evidence
    )
    effective_score = 0.5 + maturity * (score_ema - 0.5)
    track.depth_surface_score_ema = score_ema
    track.depth_surface_maturity = float(np.clip(maturity, 0.0, 1.0))
    track.depth_surface_score = float(np.clip(effective_score, 0.0, 1.0))
    track.depth_surface_components = {
        "transverse_curvature": ema.surface_transverse_curvature,
        "axial_curvature": ema.surface_axial_curvature,
        "curvature_anisotropy": ema.surface_curvature_anisotropy,
        "radius_estimate": ema.surface_radius_estimate_m,
        "curvature_score": ema.surface_curvature_score,
        "normal_axial_mad": ema.surface_normal_axial_mad_deg,
        "normal_jump": ema.surface_normal_jump_deg,
        "normal_consistency_score": ema.surface_normal_score,
        "object_boundary_ratio": ema.surface_object_boundary_ratio,
        "internal_cut_boundary_ratio": ema.surface_internal_cut_ratio,
        "boundary_support_score": ema.surface_boundary_score,
        "evidence": evidence,
    }


def _update_track_forearm_likelihood_v2(
    track: WristGeometryBranchTrack,
) -> None:
    """Compute the second, still-shadow-only, track likelihood."""
    ema = track.feature_ema
    length_score = _smoothstep(0.060, 0.140, ema.length_m)
    continuity_score = float(np.clip(ema.continuity or 0.0, 0.0, 1.0))
    width_cv_score = (
        math.exp(-((ema.width_cv / 0.25) ** 2))
        if ema.width_cv is not None else 0.0
    )
    width_jump_score = (
        math.exp(-((ema.max_width_jump / 0.35) ** 2))
        if ema.max_width_jump is not None else 0.0
    )
    width_temporal_ratios = [
        math.sqrt(max(0.0, variance)) / max(abs(float(width)), 1e-12)
        for width, variance in zip(
            ema.width_profile_m, ema.width_profile_variance
        )
        if width is not None
    ]
    width_temporal_cv = (
        float(np.mean(width_temporal_ratios))
        if width_temporal_ratios else math.inf
    )
    width_temporal_score = (
        math.exp(-((width_temporal_cv / 0.25) ** 2))
        if math.isfinite(width_temporal_cv) else 0.0
    )
    width_score = (
        width_cv_score * width_jump_score * width_temporal_score
    ) ** (1.0 / 3.0)
    section_score = float(np.clip(ema.section_stability or 0.0, 0.0, 1.0))
    radial_score = float(np.clip(ema.radial_progression or 0.0, 0.0, 1.0))
    branching_penalty = float(np.clip(
        ema.branching_complexity or 0.0, 0.0, 1.0
    ))

    gradient_score = _radius_gradient_score(ema.radius_gradient)
    radius_smoothness_score = float(np.clip(
        ema.radius_smoothness if ema.radius_smoothness is not None else 0.5,
        0.0,
        1.0,
    ))
    radius_jump_score = (
        math.exp(-((ema.radius_jump / 0.40) ** 2))
        if ema.radius_jump is not None else 0.5
    )
    radius_shape_raw = (
        max(gradient_score, 1e-12) ** 0.30
        * max(radius_smoothness_score, 1e-12) ** 0.40
        * max(radius_jump_score, 1e-12) ** 0.30
    )
    radius_evidence = _smoothstep(0.40, 0.80, ema.radius_valid_ratio)
    radius_shape_score = float(
        0.5 + radius_evidence * (radius_shape_raw - 0.5)
    )

    axis_jitter = None
    axis_stability_raw = None
    if track.axis_delta_window_deg:
        axis_deltas = np.asarray(track.axis_delta_window_deg, dtype=np.float64)
        median_delta = float(np.median(axis_deltas))
        median_absolute_deviation = float(np.median(
            np.abs(axis_deltas - median_delta)
        ))
        axis_jitter = median_delta + 1.4826 * median_absolute_deviation
        axis_stability_raw = math.exp(-((axis_jitter / 8.0) ** 2))

    feature_values = (
        ema.length_m,
        ema.continuity,
        ema.width_cv,
        ema.max_width_jump,
        ema.section_stability,
        ema.branching_complexity,
        ema.radial_progression,
        ema.radius_gradient,
        ema.radius_smoothness,
        ema.radius_jump,
    )
    feature_coverage = (
        sum(value is not None for value in feature_values) / len(feature_values)
    )
    sample_evidence = _smoothstep(1.0, 8.0, float(ema.update_count))
    visibility_evidence = float(np.clip(
        track.hit_count / max(1, track.age_frames), 0.0, 1.0
    ))
    consecutive_evidence = _smoothstep(
        1.0, 8.0, float(track.consecutive_hits)
    )
    observation_confidence = float(np.clip(
        0.30 * sample_evidence
        + 0.25 * visibility_evidence
        + 0.20 * consecutive_evidence
        + 0.15 * feature_coverage
        + 0.10 * float(np.clip(ema.radius_valid_ratio or 0.0, 0.0, 1.0)),
        0.0,
        1.0,
    ))
    age_evidence = _smoothstep(3.0, 12.0, float(ema.update_count))
    confidence_evidence = _smoothstep(0.35, 0.75, observation_confidence)
    axis_maturity = math.sqrt(age_evidence * confidence_evidence)
    axis_stability_effective = float(
        0.5 + axis_maturity * (
            (axis_stability_raw if axis_stability_raw is not None else 0.5) - 0.5
        )
    )

    components = {
        "length": 0.18 * length_score,
        "continuity": 0.08 * continuity_score,
        "width": 0.14 * width_score,
        "section": 0.05 * section_score,
        "axis": 0.23 * axis_stability_effective,
        "radial": 0.12 * radial_score,
        "radius_shape": 0.20 * radius_shape_score,
        "branch_penalty": -0.10 * branching_penalty,
    }
    track.forearm_score_v2 = float(np.clip(sum(components.values()), 0.0, 1.0))
    track.confidence_v2 = float(np.clip(
        observation_confidence
        * math.sqrt(float(np.clip(ema.update_count / 8.0, 0.0, 1.0))),
        0.0,
        1.0,
    ))
    track.score_v2_components = components
    track.radius_components = {
        "profile": ema.radius_profile_m,
        "gradient": ema.radius_gradient,
        "gradient_score": gradient_score,
        "smoothness": ema.radius_smoothness,
        "smoothness_score": radius_smoothness_score,
        "jump": ema.radius_jump,
        "jump_score": radius_jump_score,
        "valid_ratio": ema.radius_valid_ratio,
        "evidence": radius_evidence,
        "shape_score_raw": radius_shape_raw,
        "shape_score": radius_shape_score,
    }
    track.axis_jitter_robust_deg = axis_jitter
    track.axis_stability_raw = axis_stability_raw
    track.axis_maturity = float(axis_maturity)
    track.axis_stability_effective = axis_stability_effective
    _update_track_topology_score(track)
    _update_track_depth_surface_score(track)


def _update_track_forearm_likelihood(track: WristGeometryBranchTrack) -> None:
    """Compute a shadow likelihood strictly from the track's EMA features."""
    ema = track.feature_ema
    length_score = _smoothstep(0.060, 0.140, ema.length_m)
    continuity_score = float(np.clip(ema.continuity or 0.0, 0.0, 1.0))
    width_cv_score = (
        math.exp(-((ema.width_cv / 0.25) ** 2))
        if ema.width_cv is not None else 0.0
    )
    width_jump_score = (
        math.exp(-((ema.max_width_jump / 0.35) ** 2))
        if ema.max_width_jump is not None else 0.0
    )
    width_temporal_ratios = [
        math.sqrt(max(0.0, variance)) / max(abs(float(width)), 1e-12)
        for width, variance in zip(
            ema.width_profile_m, ema.width_profile_variance
        )
        if width is not None
    ]
    width_temporal_cv = (
        float(np.mean(width_temporal_ratios))
        if width_temporal_ratios else math.inf
    )
    width_temporal_score = (
        math.exp(-((width_temporal_cv / 0.25) ** 2))
        if math.isfinite(width_temporal_cv) else 0.0
    )
    width_score = (width_cv_score * width_jump_score * width_temporal_score) ** (1.0 / 3.0)
    section_score = float(np.clip(ema.section_stability or 0.0, 0.0, 1.0))
    axis_jitter = (
        ema.axis_delta_deg + 2.0 * math.sqrt(max(0.0, ema.axis_delta_variance))
        if ema.axis_delta_deg is not None else math.inf
    )
    axis_score = math.exp(-((axis_jitter / 5.0) ** 2)) if math.isfinite(axis_jitter) else 0.0
    radial_score = float(np.clip(ema.radial_progression or 0.0, 0.0, 1.0))
    branching_penalty = float(np.clip(ema.branching_complexity or 0.0, 0.0, 1.0))
    track.forearm_score = float(np.clip(
        0.15 * length_score
        + 0.22 * continuity_score
        + 0.18 * width_score
        + 0.13 * section_score
        + 0.20 * axis_score
        + 0.12 * radial_score
        - 0.12 * branching_penalty,
        0.0,
        1.0,
    ))

    feature_values = (
        ema.length_m,
        ema.continuity,
        ema.width_cv,
        ema.max_width_jump,
        ema.section_stability,
        ema.branching_complexity,
        ema.axis_delta_deg,
        ema.radial_progression,
    )
    feature_coverage = sum(value is not None for value in feature_values) / len(feature_values)
    sample_confidence = float(np.clip(ema.update_count / 8.0, 0.0, 1.0))
    visibility_confidence = float(np.clip(track.hit_count / max(1, track.age_frames), 0.0, 1.0))
    consecutive_confidence = float(np.clip(track.consecutive_hits / 8.0, 0.0, 1.0))
    temporal_confidence = axis_score
    confidence_evidence = (
        0.25 * sample_confidence
        + 0.20 * visibility_confidence
        + 0.20 * feature_coverage
        + 0.20 * temporal_confidence
        + 0.15 * consecutive_confidence
    )
    track.confidence = float(np.clip(
        confidence_evidence * math.sqrt(sample_confidence),
        0.0,
        1.0,
    ))
    _update_track_forearm_likelihood_v2(track)


def update_wrist_geometry_branch_tracks(
    estimate: WristGeometryAxisEstimate,
    state: WristGeometryBranchTrackerState,
    timestamp_sec: float | None = None,
) -> WristGeometryBranchTrackingEstimate:
    """Associate V2.1 branches without changing which branch V2.1 selects."""
    state.last_lost_events = []
    for track in state.tracks.values():
        track.age_frames += 1
        track.current_branch_id = None
        track.last_match_cost = None

    feature_branches = [
        branch for branch in estimate.branches if branch.features is not None
    ]
    pair_candidates: list[tuple[float, int, int]] = []
    for track_id, track in state.tracks.items():
        for branch in feature_branches:
            cost = _wrist_branch_descriptor_match_cost(
                track.descriptor, branch.features.descriptor
            )
            if cost is not None and cost <= 1.0:
                pair_candidates.append((cost, track_id, branch.branch_id))
    pair_candidates.sort(key=lambda item: item[0])

    branch_to_track: dict[int, int] = {}
    branch_match_cost: dict[int, float | None] = {}
    matched_tracks: set[int] = set()
    for cost, track_id, branch_id in pair_candidates:
        if track_id in matched_tracks or branch_id in branch_to_track:
            continue
        branch = next(item for item in feature_branches if item.branch_id == branch_id)
        track = state.tracks[track_id]
        previous_axis = track.descriptor.axis
        current_axis = branch.features.descriptor.axis
        axis_delta_deg = 0.0
        if previous_axis is not None and current_axis is not None:
            cosine = abs(float(np.dot(previous_axis, current_axis))) / max(
                float(np.linalg.norm(previous_axis) * np.linalg.norm(current_axis)),
                1e-12,
            )
            axis_delta_deg = float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))
            track.axis_delta_window_deg.append(axis_delta_deg)
        track.descriptor = _blend_wrist_branch_descriptor(
            track.descriptor, branch.features.descriptor
        )
        track.hit_count += 1
        track.consecutive_hits += 1
        track.missed_frames = 0
        track.current_branch_id = branch_id
        track.last_match_cost = cost
        track.last_lost_reason = None
        track.last_lost_detail = {}
        _update_track_feature_ema(
            track, branch.features, axis_delta_deg, timestamp_sec
        )
        _update_track_forearm_likelihood(track)
        branch_to_track[branch_id] = track_id
        branch_match_cost[branch_id] = cost
        matched_tracks.add(track_id)

    for track_id, track in list(state.tracks.items()):
        if track_id in matched_tracks:
            continue
        if not feature_branches:
            track.last_lost_reason = "ESTIMATOR_NO_BRANCH"
            track.last_lost_detail = {"estimator_reason": estimate.reason}
        else:
            match_diagnostics = []
            for branch in feature_branches:
                cost, reason, details = _wrist_branch_descriptor_match_diagnostic(
                    track.descriptor, branch.features.descriptor
                )
                normalized_distance = (
                    details.get("axis_angle_deg", 35.0) / 35.0
                    + details.get("centroid_distance_m", 0.10) / 0.10
                    + details.get("far_endpoint_distance_m", 0.12) / 0.12
                    + details.get("length_relative_delta", 0.70) / 0.70
                )
                match_diagnostics.append((
                    normalized_distance,
                    branch.branch_id,
                    cost,
                    reason,
                    details,
                ))
            admissible = [
                item for item in match_diagnostics
                if item[2] is not None and item[2] <= 1.0
            ]
            if admissible:
                closest = min(admissible, key=lambda item: (item[2], item[1]))
                track.last_lost_reason = "MATCH_COMPETITION_LOST"
            else:
                closest = min(match_diagnostics, key=lambda item: (item[0], item[1]))
                track.last_lost_reason = closest[3]
            track.last_lost_detail = {
                "best_branch_id": closest[1],
                "best_cost": closest[2],
                **closest[4],
            }
        track.missed_frames += 1
        track.consecutive_hits = 0
        track.confidence *= math.exp(-1.0 / max(1, state.maximum_missed_frames))
        track.confidence_v2 *= math.exp(
            -1.0 / max(1, state.maximum_missed_frames)
        )
        if track.missed_frames > state.maximum_missed_frames:
            state.last_lost_events.append({
                "track_id": track.track_id,
                "reason": "TRACK_EXPIRED",
                "last_lost_reason": track.last_lost_reason,
                "missed_frames": track.missed_frames,
                "detail": dict(track.last_lost_detail),
            })
            del state.tracks[track_id]

    for branch in feature_branches:
        if branch.branch_id in branch_to_track:
            continue
        track_id = state.next_track_id
        state.next_track_id += 1
        state.tracks[track_id] = WristGeometryBranchTrack(
            track_id=track_id,
            descriptor=branch.features.descriptor,
            current_branch_id=branch.branch_id,
        )
        _update_track_feature_ema(
            state.tracks[track_id], branch.features, 0.0, timestamp_sec
        )
        _update_track_forearm_likelihood(state.tracks[track_id])
        branch_to_track[branch.branch_id] = track_id
        branch_match_cost[branch.branch_id] = None

    candidate_track_id = (
        branch_to_track.get(estimate.selected_branch)
        if estimate.selected_branch is not None else None
    )
    visible_track_ids = {
        track_id for track_id, track in state.tracks.items()
        if track.current_branch_id is not None
    }
    if state.active_track_id not in state.tracks:
        state.active_track_id = None
    active_state = "NO_ACTIVE_TRACK"
    if state.active_track_id is not None:
        if state.active_track_id in visible_track_ids:
            active_state = (
                "TRACKING"
                if candidate_track_id in (None, state.active_track_id)
                else "TRACKING_CANDIDATE_DIFFERENT"
            )
        else:
            active_state = "ACTIVE_MISSING"
    elif candidate_track_id is not None:
        candidate_track = state.tracks[candidate_track_id]
        if candidate_track.consecutive_hits >= state.minimum_confirmation_frames:
            state.active_track_id = candidate_track_id
            active_state = "TRACKING"
        else:
            active_state = "WAITING_FOR_CONFIRMATION"

    tracked_branches = []
    for branch in estimate.branches:
        track_id = branch_to_track.get(branch.branch_id)
        track = state.tracks.get(track_id) if track_id is not None else None
        tracked_branches.append(WristGeometryTrackedBranch(
            branch_id=branch.branch_id,
            track_id=track_id,
            match_cost=branch_match_cost.get(branch.branch_id),
            track_age_frames=track.age_frames if track is not None else 0,
            consecutive_hits=track.consecutive_hits if track is not None else 0,
            confirmed=(
                track is not None
                and track.consecutive_hits >= state.minimum_confirmation_frames
            ),
            valid=branch.valid,
            reason=branch.reason,
            features=branch.features,
        ))
    candidate_tracks = tuple(
        branch_to_track[branch.branch_id]
        for branch in sorted(
            feature_branches,
            key=lambda item: (
                not item.valid,
                -(item.features.effective_length_m if item.features else 0.0),
                item.branch_id,
            ),
        )
    )
    track_likelihoods = tuple(
        WristGeometryTrackLikelihood(
            track_id=track.track_id,
            visible=track.current_branch_id is not None,
            age_frames=track.age_frames,
            consecutive_hits=track.consecutive_hits,
            missed_frames=track.missed_frames,
            ema_feature=track.feature_ema.diagnostics(),
            forearm_score=track.forearm_score,
            confidence=track.confidence,
            forearm_score_v2=track.forearm_score_v2,
            confidence_v2=track.confidence_v2,
            score_v2_components=dict(track.score_v2_components),
            radius_components=dict(track.radius_components),
            axis_jitter_robust_deg=track.axis_jitter_robust_deg,
            axis_stability_raw=track.axis_stability_raw,
            axis_maturity=track.axis_maturity,
            axis_stability_effective=track.axis_stability_effective,
            topology_score=track.topology_score,
            topology_score_raw=track.topology_score_raw,
            topology_maturity=track.topology_maturity,
            forearm_score_topology=track.forearm_score_topology,
            topology_components=dict(track.topology_components),
            track_lost_reason=track.last_lost_reason,
            track_lost_detail=dict(track.last_lost_detail),
            depth_surface_score_raw=track.depth_surface_score_raw,
            depth_surface_score_ema=track.depth_surface_score_ema,
            depth_surface_maturity=track.depth_surface_maturity,
            depth_surface_score=track.depth_surface_score,
            depth_surface_components=dict(track.depth_surface_components),
        )
        for track in sorted(state.tracks.values(), key=lambda item: item.track_id)
    )
    score_v2_candidates = sorted(
        (
            track for track in state.tracks.values()
            if track.current_branch_id is not None
            and track.consecutive_hits >= state.minimum_confirmation_frames
        ),
        key=lambda item: (-item.forearm_score_v2, item.track_id),
    )
    best_track = score_v2_candidates[0] if score_v2_candidates else None
    second_track = (
        score_v2_candidates[1] if len(score_v2_candidates) >= 2 else None
    )
    if best_track is None:
        state.score_v2_best_track_id = None
        state.score_v2_best_streak_frames = 0
    elif state.score_v2_best_track_id == best_track.track_id:
        state.score_v2_best_streak_frames += 1
    else:
        state.score_v2_best_track_id = best_track.track_id
        state.score_v2_best_streak_frames = 1
    raw_margin = None
    reliable_margin = None
    if best_track is not None and second_track is not None:
        raw_margin = best_track.forearm_score_v2 - second_track.forearm_score_v2
        reliable_margin = raw_margin * math.sqrt(
            best_track.confidence_v2 * second_track.confidence_v2
        )
    score_v2_shadow = {
        "best_track_id": best_track.track_id if best_track is not None else None,
        "best_score": (
            best_track.forearm_score_v2 if best_track is not None else None
        ),
        "best_confidence": (
            best_track.confidence_v2 if best_track is not None else None
        ),
        "second_track_id": (
            second_track.track_id if second_track is not None else None
        ),
        "second_score": (
            second_track.forearm_score_v2 if second_track is not None else None
        ),
        "second_confidence": (
            second_track.confidence_v2 if second_track is not None else None
        ),
        "raw_margin": raw_margin,
        "reliable_margin": reliable_margin,
        "candidate_count": len(score_v2_candidates),
        "best_streak_frames": state.score_v2_best_streak_frames,
        "reason": "SHADOW_ONLY",
    }
    return WristGeometryBranchTrackingEstimate(
        branches=tuple(tracked_branches),
        tracks=track_likelihoods,
        active_track_id=state.active_track_id,
        active_state=active_state,
        candidate_track_id=candidate_track_id,
        candidate_tracks=candidate_tracks,
        score_v2_shadow=score_v2_shadow,
        track_lost_events=tuple(state.last_lost_events),
    )


def refine_forearm_axis(
    sections: list[ForearmSection],
    inlier_indices: np.ndarray,
    reference_direction: ArmDirection | None = None,
    minimum_axis_length_m: float = 0.040,
    maximum_rms_m: float = 0.008,
    minimum_inlier_ratio: float = 0.60,
    minimum_pca_ratio: float = 2.5,
    minimum_confidence: float = 0.40,
) -> ForearmAxisFit:
    """PCA-refine RANSAC inliers and classify strong/degraded/rejected."""
    section_count = len(sections)
    inlier_set = {int(index) for index in inlier_indices}
    marked_sections = tuple(
        replace(section, inlier=section.valid and section.section_index in inlier_set)
        for section in sections
    )
    valid_section_count = sum(section.valid for section in marked_sections)
    inlier_sections = [section for section in marked_sections if section.inlier]
    inlier_ratio = float(len(inlier_sections) / max(1, valid_section_count))

    runs: list[list[ForearmSection]] = []
    for section in sorted(inlier_sections, key=lambda item: item.section_index):
        if not runs or section.section_index != runs[-1][-1].section_index + 1:
            runs.append([section])
        else:
            runs[-1].append(section)
    max_consecutive = max((len(run) for run in runs), default=0)

    def result(
        reason: str,
        quality_level: str = "PRIMARY_REJECTED",
        direction: ArmDirection | None = None,
        **values,
    ) -> ForearmAxisFit:
        return ForearmAxisFit(
            direction=direction,
            origin_m=values.get("origin_m"),
            axis_length_m=values.get("axis_length_m"),
            rms_m=values.get("rms_m"),
            confidence=float(values.get("confidence", 0.0)),
            section_count=section_count,
            inlier_section_count=len(inlier_sections),
            inlier_ratio=inlier_ratio,
            pca_ratio=values.get("pca_ratio"),
            valid=quality_level == "PRIMARY_STRONG",
            reason=reason,
            quality_level=quality_level,
            valid_section_count=valid_section_count,
            max_consecutive_inlier_sections=max_consecutive,
            max_consecutive_inlier_span_m=float(
                values.get("max_consecutive_inlier_span_m", 0.0)
            ),
            sections=marked_sections,
        )

    if len(inlier_sections) < 4:
        return result("TOO_FEW_INLIERS")
    centers = np.asarray(
        [section.center_m for section in inlier_sections], dtype=np.float64
    )
    origin = np.mean(centers, axis=0)
    centered = centers - origin
    covariance = centered.T @ centered / max(1, centers.shape[0] - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 1e-10:
        return result("PCA_DEGENERATE", origin_m=tuple(origin))
    direction = eigenvectors[:, -1]
    ordered = sorted(
        (section for section in sections if section.valid and section.center_m is not None),
        key=lambda section: section.longitudinal_px,
    )
    section_orientation = np.asarray(ordered[-1].center_m) - np.asarray(ordered[0].center_m)
    if reference_direction is not None:
        orientation = np.asarray(reference_direction.vector, dtype=np.float64)
    else:
        orientation = section_orientation
    if float(np.dot(direction, orientation)) < 0.0:
        direction = -direction
    projections = centered @ direction
    axis_length_m = float(np.max(projections) - np.min(projections))
    residuals = np.linalg.norm(centered - np.outer(projections, direction), axis=1)
    rms_m = float(math.sqrt(np.mean(np.square(residuals))))
    pca_ratio = float(eigenvalues[-1] / max(eigenvalues[-2], 1e-12))
    max_consecutive_span_m = 0.0
    for run in runs:
        run_centers = np.asarray([section.center_m for section in run])
        run_projections = (run_centers - origin) @ direction
        max_consecutive_span_m = max(
            max_consecutive_span_m,
            float(np.max(run_projections) - np.min(run_projections)),
        )
    length_score = float(np.clip((axis_length_m - 0.025) / 0.075, 0.0, 1.0))
    residual_score = float(np.clip(1.0 - rms_m / maximum_rms_m, 0.0, 1.0))
    section_score = float(np.clip((inlier_indices.size - 3) / 5.0, 0.0, 1.0))
    pca_score = float(np.clip((pca_ratio - 1.0) / 7.0, 0.0, 1.0))
    confidence = float(np.clip(
        0.25 * length_score
        + 0.30 * residual_score
        + 0.20 * inlier_ratio
        + 0.10 * section_score
        + 0.15 * pca_score,
        0.0, 1.0,
    ))
    diagnostics = {
        "origin_m": tuple(float(value) for value in origin),
        "axis_length_m": axis_length_m,
        "rms_m": rms_m,
        "confidence": confidence,
        "pca_ratio": pca_ratio,
        "max_consecutive_inlier_span_m": max_consecutive_span_m,
    }
    axis = ArmDirection(tuple(float(value) for value in direction))
    if axis_length_m < minimum_axis_length_m:
        return result("TOO_SHORT", direction=axis, **diagnostics)
    if rms_m > maximum_rms_m:
        return result("RMS_TOO_HIGH", direction=axis, **diagnostics)
    if pca_ratio < minimum_pca_ratio:
        return result("PCA_DEGENERATE", direction=axis, **diagnostics)
    if confidence < minimum_confidence:
        return result("LOW_CONFIDENCE", direction=axis, **diagnostics)
    if inlier_ratio >= minimum_inlier_ratio:
        return result(
            "NONE", quality_level="PRIMARY_STRONG", direction=axis, **diagnostics
        )
    degraded = (
        inlier_ratio >= 0.44
        and max_consecutive >= 4
        and max_consecutive_span_m >= 0.060
        and axis_length_m >= 0.060
        and rms_m <= 0.005
        and pca_ratio >= 3.0
        and confidence >= 0.55
    )
    if degraded:
        return result(
            "INLIER_RATIO_LOW",
            quality_level="PRIMARY_DEGRADED",
            direction=axis,
            **diagnostics,
        )
    continuity_reason = (
        "INLIER_CONTINUITY_LOW"
        if inlier_ratio >= 0.44 else "INLIER_RATIO_LOW"
    )
    return result(continuity_reason, direction=axis, **diagnostics)


def estimate_forearm_axis_3d_diagnostic(
    landmarks,
    arm_mask: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
    reference_direction: ArmDirection | None = None,
) -> ForearmAxisFit:
    """Estimate the primary 3D forearm axis from mask/depth section centers."""
    def empty(reason: str, sections=(), inlier_count: int = 0) -> ForearmAxisFit:
        valid_count = sum(section.valid for section in sections)
        return ForearmAxisFit(
            direction=None,
            origin_m=None,
            axis_length_m=None,
            rms_m=None,
            confidence=0.0,
            section_count=len(sections),
            inlier_section_count=inlier_count,
            inlier_ratio=float(inlier_count / max(1, valid_count)),
            pca_ratio=None,
            valid=False,
            reason=reason,
            quality_level="PRIMARY_REJECTED",
            valid_section_count=valid_count,
            sections=tuple(sections),
        )
    if arm_mask.shape != depth_m.shape or not np.any(arm_mask > 0):
        return empty("MASK_EMPTY")
    corridor, wrist, image_axis, palm_width_px = select_forearm_corridor(
        arm_mask, landmarks
    )
    if palm_width_px < 8.0 or not np.any(corridor > 0):
        return empty("CORRIDOR_EMPTY")
    points, pixels = masked_depth_to_points(
        corridor, depth_m, intrinsics, mirrored=mirrored
    )
    if points.shape[0] < 80:
        return empty("DEPTH_INSUFFICIENT")
    sections = compute_forearm_sections(points, pixels, wrist, image_axis)
    _origin, _axis, inliers, reason = fit_forearm_centerline_ransac(sections)
    if reason != "NONE":
        return empty(reason, sections, int(inliers.size))
    return refine_forearm_axis(
        sections, inliers, reference_direction=reference_direction
    )


def _unsigned_axis_angle_deg(
    first: ArmDirection | None, second: ArmDirection | None
) -> float | None:
    if first is None or second is None:
        return None
    first_vector = np.asarray(first.vector, dtype=np.float64)
    second_vector = np.asarray(second.vector, dtype=np.float64)
    first_length = float(np.linalg.norm(first_vector))
    second_length = float(np.linalg.norm(second_vector))
    if first_length < 1e-9 or second_length < 1e-9:
        return None
    cosine = abs(float(np.dot(
        first_vector / first_length, second_vector / second_length
    )))
    return math.degrees(math.acos(float(np.clip(cosine, 0.0, 1.0))))


def select_forearm_axis(
    landmarks,
    arm_mask: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics | None,
    state: ForearmAxisSelectorState,
    mirrored: bool = False,
) -> ForearmAxisEstimate:
    """Hysteretically select PRIMARY, fallback, or invalid without blending."""
    fallback = (
        estimate_arm_direction(landmarks, depth_m, intrinsics, mirrored)
        if landmarks is not None and intrinsics is not None else None
    )
    if landmarks is not None and intrinsics is not None:
        primary = estimate_forearm_axis_3d_diagnostic(
            landmarks, arm_mask, depth_m, intrinsics, mirrored,
            reference_direction=fallback,
        )
    else:
        primary = ForearmAxisFit(
            direction=None, origin_m=None, axis_length_m=None, rms_m=None,
            confidence=0.0, section_count=0, inlier_section_count=0,
            inlier_ratio=0.0, pca_ratio=None, valid=False,
            reason="INVALID_INPUT",
        )
    primary_strong = (
        primary.quality_level == "PRIMARY_STRONG"
        and primary.direction is not None
    )
    candidate_source = (
        "PRIMARY_3D_FOREARM" if primary_strong
        else "FALLBACK_MEDIAPIPE" if fallback is not None
        else "INVALID"
    )
    unsigned_angle = _unsigned_axis_angle_deg(primary.direction, fallback)
    axis_delta_prev = _unsigned_axis_angle_deg(
        primary.direction, state.last_primary_direction
    )
    if primary.direction is not None:
        state.last_primary_direction = primary.direction

    source_changed = False
    switch_reason = "NONE"

    def commit(source: str, reason: str) -> None:
        nonlocal source_changed, switch_reason
        if source == state.active_source:
            return
        state.active_source = source
        state.source_switch_count += 1
        source_changed = True
        switch_reason = reason

    direction = None
    if state.active_source == "INVALID":
        state.primary_failure_streak = 0
        if candidate_source == "PRIMARY_3D_FOREARM":
            state.primary_success_streak += 1
            state.selector_state = "PRIMARY_PROBATION"
            if state.primary_success_streak >= state.primary_confirm_frames:
                commit("PRIMARY_3D_FOREARM", "PRIMARY_STRONG_CONFIRMED")
                state.selector_state = "ACTIVE_PRIMARY"
                direction = primary.direction
            elif fallback is not None:
                commit("FALLBACK_MEDIAPIPE", "INITIAL_FALLBACK_LOCK")
                direction = fallback
        elif candidate_source == "FALLBACK_MEDIAPIPE":
            state.primary_success_streak = 0
            commit("FALLBACK_MEDIAPIPE", "INITIAL_FALLBACK_LOCK")
            state.selector_state = (
                "PRIMARY_DEGRADED_SHADOW"
                if primary.quality_level == "PRIMARY_DEGRADED"
                else "ACTIVE_FALLBACK"
            )
            direction = fallback
        else:
            state.primary_success_streak = 0
            state.selector_state = "WAITING_FOR_INPUT"
    elif state.active_source == "FALLBACK_MEDIAPIPE":
        state.primary_failure_streak = 0
        if candidate_source == "PRIMARY_3D_FOREARM":
            state.primary_success_streak += 1
            if state.primary_success_streak >= state.primary_confirm_frames:
                commit("PRIMARY_3D_FOREARM", "PRIMARY_STRONG_CONFIRMED")
                state.selector_state = "ACTIVE_PRIMARY"
                direction = primary.direction
            else:
                state.selector_state = "PRIMARY_PROBATION"
                direction = fallback
        elif candidate_source == "FALLBACK_MEDIAPIPE":
            state.primary_success_streak = 0
            state.selector_state = (
                "PRIMARY_DEGRADED_SHADOW"
                if primary.quality_level == "PRIMARY_DEGRADED"
                else "ACTIVE_FALLBACK"
            )
            direction = fallback
        else:
            state.primary_success_streak = 0
            commit("INVALID", "FALLBACK_UNAVAILABLE")
            state.selector_state = "WAITING_FOR_INPUT"
    else:  # ACTIVE PRIMARY
        state.primary_success_streak = 0
        if candidate_source == "PRIMARY_3D_FOREARM":
            state.primary_failure_streak = 0
            state.selector_state = "ACTIVE_PRIMARY"
            direction = primary.direction
        else:
            state.primary_failure_streak += 1
            if state.primary_failure_streak < state.primary_failure_frames:
                state.selector_state = "PRIMARY_HOLD"
                # Do not blend fallback into a PRIMARY filter and do not
                # republish a stale PRIMARY vector with a fresh timestamp.
                direction = None
            elif fallback is not None:
                commit("FALLBACK_MEDIAPIPE", "PRIMARY_FAILURE_CONFIRMED")
                state.selector_state = (
                    "PRIMARY_DEGRADED_SHADOW"
                    if primary.quality_level == "PRIMARY_DEGRADED"
                    else "ACTIVE_FALLBACK"
                )
                state.primary_failure_streak = 0
                direction = fallback
            else:
                commit("INVALID", "PRIMARY_AND_FALLBACK_UNAVAILABLE")
                state.selector_state = "WAITING_FOR_INPUT"
                state.primary_failure_streak = 0

    if state.active_source == "PRIMARY_3D_FOREARM":
        confidence = primary.confidence
    elif state.active_source == "FALLBACK_MEDIAPIPE":
        confidence = 0.50
    else:
        confidence = 0.0
    reason = (
        "NONE" if primary_strong
        else primary.reason if fallback is not None
        else f"{primary.reason};FALLBACK_UNAVAILABLE"
    )
    state.last_reason = reason
    return ForearmAxisEstimate(
        direction=direction,
        source=state.active_source,
        axis_length_m=primary.axis_length_m,
        rms_m=primary.rms_m,
        confidence=confidence,
        reason=reason,
        source_changed=source_changed,
        candidate_source=candidate_source,
        active_source=state.active_source,
        selector_state=state.selector_state,
        switch_reason=switch_reason,
        primary_direction=primary.direction,
        mediapipe_direction=fallback,
        unsigned_angle_deg=unsigned_angle,
        axis_delta_prev_deg=axis_delta_prev,
        primary_quality_level=primary.quality_level,
        valid_section_count=primary.valid_section_count,
        inlier_section_count=primary.inlier_section_count,
        inlier_ratio=primary.inlier_ratio,
        max_consecutive_inlier_sections=(
            primary.max_consecutive_inlier_sections
        ),
        max_consecutive_inlier_span_m=(
            primary.max_consecutive_inlier_span_m
        ),
        pca_ratio=primary.pca_ratio,
    )


def estimate_surface_patch_diagnostic(
    depth_m: np.ndarray,
    pixel: tuple[int, int],
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
    radius_px: int = 18,
    maximum_depth_deviation_m: float = 0.015,
    maximum_rms_m: float = 0.004,
    ransac_distance_m: float = 0.0025,
    minimum_inlier_count: int = 30,
    minimum_inlier_ratio: float = 0.55,
    minimum_normal_confidence: float = 0.35,
    valid_mask: np.ndarray | None = None,
    reference_depth_m: float | None = None,
) -> tuple[SurfacePatch | None, str, dict]:
    """RANSAC-fit a local RGB-D skin plane and report its quality.

    The normal is sign-oriented along the camera ray (camera -> skin).  With
    ``P_goal = P_surface - normal * standoff`` this places Link6 on the camera
    side of the skin instead of inside it.
    """
    height, width = depth_m.shape
    x, y = pixel
    radius_px = max(6, int(radius_px))
    x0, x1 = max(0, x - radius_px), min(width, x + radius_px + 1)
    y0, y1 = max(0, y - radius_px), min(height, y + radius_px + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = np.isfinite(patch) & (patch >= 0.10) & (patch <= 2.0)
    if valid_mask is not None:
        if valid_mask.shape != depth_m.shape:
            raise ValueError("surface-normal mask must match the depth image")
        valid &= valid_mask[y0:y1, x0:x1] > 0
    roi_count = int(patch.size)
    initially_valid = int(np.count_nonzero(valid))
    depth_valid_ratio = initially_valid / max(1, roi_count)
    details = {
        "plane_inlier_count": 0,
        "plane_residual": None,
        "normal_confidence": 0.0,
        "depth_valid_ratio": depth_valid_ratio,
        "normal_angle": None,
    }
    if initially_valid < minimum_inlier_count:
        return None, "PLANE_INSUFFICIENT_DEPTH", details
    reference_depth = (
        float(reference_depth_m)
        if reference_depth_m is not None and math.isfinite(reference_depth_m)
        else float(np.median(patch[valid]))
    )
    valid &= np.abs(patch - reference_depth) <= maximum_depth_deviation_m
    rows, cols = np.nonzero(valid)
    if rows.size < minimum_inlier_count:
        return None, "PLANE_BACKGROUND_REJECTION", details
    # Subsample dense patches without biasing one side of the plane.
    stride = max(1, int(math.ceil(rows.size / 600.0)))
    points = np.asarray([
        deproject_pixel(
            (x0 + int(col), y0 + int(row)),
            float(patch[row, col]), intrinsics, width, height, mirrored,
        )
        for row, col in zip(rows[::stride], cols[::stride])
    ], dtype=np.float64)
    if points.shape[0] < minimum_inlier_count:
        return None, "PLANE_INSUFFICIENT_DEPTH", details

    # Deterministic RANSAC removes fingers, sleeves and background pixels that
    # survive the coarse depth band.  Determinism keeps diagnostics/test runs
    # comparable for an identical RGB-D frame.
    rng = np.random.default_rng(0)
    best_indices = np.empty(0, dtype=np.int64)
    best_residual = math.inf
    for _ in range(96):
        sample = points[rng.choice(points.shape[0], size=3, replace=False)]
        candidate_normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = float(np.linalg.norm(candidate_normal))
        if not math.isfinite(length) or length < 1e-9:
            continue
        candidate_normal /= length
        distances = np.abs((points - sample[0]) @ candidate_normal)
        indices = np.flatnonzero(distances <= ransac_distance_m)
        residual = float(np.median(distances[indices])) if indices.size else math.inf
        if indices.size > best_indices.size or (
            indices.size == best_indices.size and residual < best_residual
        ):
            best_indices, best_residual = indices, residual

    inlier_ratio = best_indices.size / max(1, points.shape[0])
    details["plane_inlier_count"] = int(best_indices.size)
    if best_indices.size < minimum_inlier_count or inlier_ratio < minimum_inlier_ratio:
        return None, "PLANE_TOO_FEW_INLIERS", details

    inliers = points[best_indices]
    center = np.mean(inliers, axis=0)
    centered = inliers - center
    covariance = centered.T @ centered / max(1, inliers.shape[0] - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[1] <= 1e-10:
        return None, "PLANE_DEGENERATE", details
    normal = eigenvectors[:, 0]
    distances = np.abs(centered @ normal)
    rms = float(math.sqrt(np.mean(np.square(distances))))
    planarity = max(0.0, 1.0 - float(eigenvalues[0] / eigenvalues[1]) / 0.25)
    residual_score = max(0.0, 1.0 - rms / maximum_rms_m)
    normal_confidence = float(np.clip(inlier_ratio * planarity * residual_score, 0.0, 1.0))
    details.update(
        plane_residual=rms,
        normal_confidence=normal_confidence,
    )
    # Reject curved/noisy patches which do not define a trustworthy plane.
    if rms > maximum_rms_m or eigenvalues[0] / eigenvalues[1] > 0.25:
        return None, "PLANE_RESIDUAL_TOO_HIGH", details
    if normal_confidence < minimum_normal_confidence:
        return None, "PLANE_LOW_CONFIDENCE", details

    # Intersect the fitted plane with the ray through the unchanged hand target
    # pixel. This makes the published surface point come from the same plane as
    # the approach direction instead of an unrelated median-depth sample.
    ray = np.asarray(
        deproject_pixel(pixel, 1.0, intrinsics, width, height, mirrored),
        dtype=np.float64,
    )
    denominator = float(np.dot(normal, ray))
    if abs(denominator) < 1e-6:
        return None, "PLANE_RAY_PARALLEL", details
    ray_scale = float(np.dot(normal, center) / denominator)
    surface_point = ray * ray_scale
    if ray_scale <= 0.0 or not np.all(np.isfinite(surface_point)):
        return None, "PLANE_INVALID_INTERSECTION", details

    camera_ray = surface_point / np.linalg.norm(surface_point)
    # Resolve the plane's +/- ambiguity. The normal points from the optical
    # origin into the surface; subtracting it is therefore the outward move.
    if float(np.dot(normal, camera_ray)) < 0.0:
        normal = -normal
    normal /= np.linalg.norm(normal)
    normal_angle_deg = math.degrees(math.acos(float(np.clip(
        np.dot(normal, camera_ray), -1.0, 1.0
    ))))
    details["normal_angle"] = normal_angle_deg
    result = SurfacePatch(
        point_m=tuple(float(value) for value in surface_point),
        normal=SurfaceNormal(
            tuple(float(value) for value in normal), rms, int(best_indices.size)
        ),
        plane_inlier_count=int(best_indices.size),
        plane_residual_m=rms,
        normal_confidence=normal_confidence,
        depth_valid_ratio=depth_valid_ratio,
        normal_angle_deg=normal_angle_deg,
    )
    return result, "NONE", details


def estimate_surface_patch(*args, **kwargs) -> SurfacePatch | None:
    """Return a quality-gated local surface estimate without diagnostics."""
    result, _reason, _details = estimate_surface_patch_diagnostic(*args, **kwargs)
    return result


def estimate_surface_normal(*args, **kwargs) -> SurfaceNormal | None:
    """Compatibility wrapper returning the normal from the local skin plane."""
    result = estimate_surface_patch(*args, **kwargs)
    return result.normal if result is not None else None


class StableSurfaceNormalFilter:
    """Require several mutually consistent, sign-oriented surface normals."""

    def __init__(self, window_size: int = 8, maximum_angle_deg: float = 5.0) -> None:
        if window_size < 3 or maximum_angle_deg <= 0.0:
            raise ValueError("surface-normal filter parameters must be positive")
        self._samples: deque[SurfaceNormal] = deque(maxlen=window_size)
        self.maximum_angle_deg = float(maximum_angle_deg)

    def clear(self) -> None:
        self._samples.clear()

    def update(self, normal: SurfaceNormal | None) -> SurfaceNormal | None:
        if normal is None:
            self.clear()
            return None
        vector = np.asarray(normal.vector, dtype=np.float64)
        vector /= np.linalg.norm(vector)
        self._samples.append(SurfaceNormal(tuple(vector), normal.rms_m, normal.point_count))
        if len(self._samples) < self._samples.maxlen:
            return None
        center = np.mean(np.asarray([sample.vector for sample in self._samples]), axis=0)
        center /= np.linalg.norm(center)
        spread = max(
            math.degrees(math.acos(float(np.clip(np.dot(sample.vector, center), -1.0, 1.0))))
            for sample in self._samples
        )
        if spread > self.maximum_angle_deg:
            return None
        return SurfaceNormal(
            tuple(float(value) for value in center),
            float(np.median([sample.rms_m for sample in self._samples])),
            int(np.median([sample.point_count for sample in self._samples])),
        )


class StableArmDirectionFilter:
    """Stabilize an unoriented 3D axis while accepting d/-d equivalence."""

    def __init__(self, window_size: int = 12, maximum_angle_deg: float = 12.0) -> None:
        if window_size < 2 or maximum_angle_deg <= 0.0:
            raise ValueError("direction filter parameters must be positive")
        self._samples: deque[np.ndarray] = deque(maxlen=window_size)
        self.maximum_angle_deg = float(maximum_angle_deg)

    def clear(self) -> None:
        self._samples.clear()

    def update(self, direction: ArmDirection | None) -> ArmDirection | None:
        if direction is None:
            self.clear()
            return None
        vector = np.asarray(direction.vector, dtype=np.float64)
        length = float(np.linalg.norm(vector))
        if not np.isfinite(length) or length < 1e-6:
            self.clear()
            return None
        vector /= length
        if self._samples and float(np.dot(vector, self._samples[0])) < 0.0:
            vector = -vector
        self._samples.append(vector)
        if len(self._samples) < self._samples.maxlen:
            return None
        center = np.mean(np.asarray(self._samples), axis=0)
        center_length = float(np.linalg.norm(center))
        if not np.isfinite(center_length) or center_length < 1e-6:
            return None
        center /= center_length
        minimum_dot = min(
            abs(float(np.dot(sample, center))) for sample in self._samples
        )
        spread = math.degrees(math.acos(float(np.clip(minimum_dot, -1.0, 1.0))))
        if spread > self.maximum_angle_deg:
            return None
        return ArmDirection(tuple(float(value) for value in center))


class StablePulseFilter:
    """P95 spatial gate over real samples; camera FPS is never a gate."""

    def __init__(
        self,
        window_sec: float = 0.6,
        stable_duration_sec: float = 0.0,
        min_samples: int = 3,
        stable_radius_m: float = 0.005,
        stable_hold_radius_m: float = 0.008,
        unstable_hold_sec: float = 0.2,
        dropout_grace_sec: float = 0.5,
        soft_timeout_sec: float = 1.5,
        hard_timeout_sec: float = 2.0,
        hard_jump_m: float = 0.015,
    ) -> None:
        if (window_sec <= 0.0 or stable_duration_sec < 0.0 or min_samples < 3
                or stable_radius_m <= 0.0
                or stable_hold_radius_m < stable_radius_m or unstable_hold_sec <= 0.0
                or not 0.0 < dropout_grace_sec < soft_timeout_sec < hard_timeout_sec
                or hard_jump_m <= stable_hold_radius_m):
            raise ValueError("invalid time-based pulse stability parameters")
        self.window_sec = float(window_sec)
        self.stable_duration_sec = float(stable_duration_sec)
        self.min_samples = int(min_samples)
        self.stable_radius_m = float(stable_radius_m)
        self.stable_hold_radius_m = float(stable_hold_radius_m)
        self.unstable_hold_sec = float(unstable_hold_sec)
        self.dropout_grace_sec = float(dropout_grace_sec)
        self.soft_timeout_sec = float(soft_timeout_sec)
        self.hard_timeout_sec = float(hard_timeout_sec)
        self.hard_jump_m = float(hard_jump_m)
        self._samples: deque[tuple[float, PulseRegion]] = deque()
        self._stable_since: float | None = None
        self._unstable_since: float | None = None
        self._last_valid_time: float | None = None
        self.frozen_region: PulseRegion | None = None
        self.stable_latched = False
        self._hand_identity: str | None = None
        self._jitter_m = 0.0
        self._drift_m = 0.0
        self._center_m: tuple[float, float, float] | None = None
        self._oldest_distance_m = 0.0
        self.acquisition_state = "WAIT_DETECTION"
        self.reset_count = 0
        self.last_reset_reason = "NONE"
        self.consecutive_missing_frames = 0
        self.reacquire_distance_m: float | None = None
        self.reacquire_result = "NONE"
        self.last_gap_sec: float | None = None
        self._last_sample_rate_hz = 0.0

    def observe_gap(self, timestamp: float) -> None:
        """Advance timeout state without inventing a missing frame or sample."""
        if not math.isfinite(timestamp):
            raise ValueError("pulse gap timestamp must be finite")
        self._expire_if_hard_timeout(timestamp)

    def clear(self, reason: str = "NEW_TARGET") -> None:
        if self._samples or self._last_valid_time is not None or self.stable_latched:
            self.reset_count += 1
            self.last_reset_reason = reason
        self._samples.clear()
        self._stable_since = None
        self._unstable_since = None
        self._last_valid_time = None
        self.frozen_region = None
        self.stable_latched = False
        self._hand_identity = None
        self._jitter_m = 0.0
        self._drift_m = 0.0
        self._center_m = None
        self._oldest_distance_m = 0.0
        self.acquisition_state = "WAIT_DETECTION"
        self.consecutive_missing_frames = 0
        self.reacquire_distance_m = None
        self.reacquire_result = "NONE"
        self.last_gap_sec = None
        self._last_sample_rate_hz = 0.0

    def _prune(self, now: float) -> None:
        while self._samples and now - self._samples[0][0] > self.window_sec:
            self._samples.popleft()

    def _expire_if_hard_timeout(self, now: float) -> None:
        if (
            self._last_valid_time is not None
            and now - self._last_valid_time > self.hard_timeout_sec
        ):
            self.clear("TARGET_HARD_TIMEOUT")

    def _window_metrics(self) -> tuple[np.ndarray, np.ndarray]:
        xyz = np.asarray(
            [sample.point_m for _, sample in self._samples], dtype=np.float64
        )
        center = np.median(xyz, axis=0)
        distances = np.linalg.norm(xyz - center, axis=1)
        self._center_m = tuple(float(value) for value in center)
        self._jitter_m = float(np.percentile(distances, 95))
        self._oldest_distance_m = float(distances[0])
        self._drift_m = float(np.linalg.norm(xyz[-1] - center))
        return center, distances

    def _release_stable_latch(self, reason: str) -> None:
        if self.stable_latched:
            self.reset_count += 1
            self.last_reset_reason = reason
        self.stable_latched = False
        self.frozen_region = None
        self._stable_since = None
        self._unstable_since = None
        self.acquisition_state = "REACQUIRING"

    def update(
        self, region: PulseRegion | None, timestamp: float,
        hand_identity: str | None = None,
    ) -> PulseRegion | None:
        if not math.isfinite(timestamp):
            raise ValueError("pulse sample timestamp must be finite")
        self._expire_if_hard_timeout(timestamp)
        if self._last_valid_time is not None and timestamp < self._last_valid_time:
            # Reject an out-of-order observation without erasing good history.
            self.reacquire_result = "OUT_OF_ORDER_SAMPLE"
            return None
        if region is None:
            self.consecutive_missing_frames += 1
            return None
        point = np.asarray(region.point_m, dtype=np.float64)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("pulse 3D point must be finite XYZ")
        if hand_identity and self._hand_identity and hand_identity != self._hand_identity:
            self.clear("HAND_IDENTITY_CHANGED")
        gap_sec = (
            timestamp - self._last_valid_time
            if self._last_valid_time is not None else None
        )
        prior_missing = self.consecutive_missing_frames
        self.last_gap_sec = gap_sec
        self.consecutive_missing_frames = 0
        self._prune(timestamp)
        if hand_identity:
            self._hand_identity = hand_identity
        prior_rate_hz = self._last_sample_rate_hz
        self._samples.append((timestamp, region))
        self._last_valid_time = timestamp
        center, _distances = self._window_metrics()
        ordinary_interval_limit = max(
            self.dropout_grace_sec / 4.0,
            2.0 / max(prior_rate_hz, 1.0),
        )
        if (
            len(self._samples) >= 2 and gap_sec is not None
            and gap_sec <= min(self.window_sec, ordinary_interval_limit)
        ):
            span = self._samples[-1][0] - self._samples[0][0]
            if span > 0.0:
                self._last_sample_rate_hz = (len(self._samples) - 1) / span
        if self.stable_latched and self.frozen_region is not None:
            self.reacquire_distance_m = float(np.linalg.norm(
                center - np.asarray(self.frozen_region.point_m)
            ))
            raw_distance_m = float(np.linalg.norm(
                point - np.asarray(self.frozen_region.point_m)
            ))
            motion_excess = self.reacquire_distance_m > self.stable_hold_radius_m
            spatial_excess = self._jitter_m > self.stable_hold_radius_m
            if (self._jitter_m > self.hard_jump_m
                    or self.reacquire_distance_m > self.hard_jump_m
                    or raw_distance_m > self.hard_jump_m):
                distance = self.reacquire_distance_m
                self.clear("TARGET_SPATIAL_DRIFT")
                self.reacquire_distance_m = distance
                self.reacquire_result = "TARGET_CHANGED"
                self.acquisition_state = "NEW_TARGET_ACQUISITION"
                # Keep this real sample as the first candidate of the new target.
                self._samples.append((timestamp, region))
                self._last_valid_time = timestamp
                if hand_identity:
                    self._hand_identity = hand_identity
                self._window_metrics()
                return None
            if spatial_excess or motion_excess:
                if self._unstable_since is None:
                    self._unstable_since = timestamp
                self.reacquire_result = "OBSERVE_SPATIAL_DRIFT"
                if timestamp - self._unstable_since >= self.unstable_hold_sec:
                    self._release_stable_latch("TARGET_SPATIAL_DRIFT")
                return None
            self._unstable_since = None
            self.reacquire_result = (
                "REACQUIRED_SAME_TARGET"
                if (
                    gap_sec is not None
                    and gap_sec > ordinary_interval_limit
                )
                or prior_missing else "CONTINUOUS_TARGET"
            )
            self.acquisition_state = "TARGET_STABLE"
            return PulseRegion(region.pixel, self._center_m, region.radius_m)
        self.acquisition_state = "NEW_TARGET_ACQUISITION"
        if len(self._samples) < self.min_samples:
            return None
        if self._jitter_m > self.stable_radius_m:
            self.acquisition_state = "ACQUISITION_CONVERGING"
            return None
        self._stable_since = timestamp
        pixels = np.asarray([sample.pixel for _, sample in self._samples], dtype=np.float64)
        radius = float(np.median([sample.radius_m for _, sample in self._samples]))
        stable_region = PulseRegion(
            pixel=tuple(int(round(value)) for value in np.median(pixels, axis=0)),
            point_m=tuple(float(value) for value in center),
            radius_m=radius,
        )
        self.frozen_region = stable_region
        self.stable_latched = True
        self.acquisition_state = "TARGET_STABLE"
        self.reacquire_result = "NEW_STABLE_TARGET"
        return stable_region

    def diagnostics(self, now: float) -> dict:
        self._expire_if_hard_timeout(now)
        recent = [item for item in self._samples if now - item[0] <= self.window_sec]
        sample_count = len(recent)
        span = recent[-1][0] - recent[0][0] if sample_count >= 2 else 0.0
        if span > 0.0:
            self._last_sample_rate_hz = (sample_count - 1) / span
        age = now - self._last_valid_time if self._last_valid_time is not None else None
        if age is None:
            target_state = (
                "TARGET_LOST" if self.last_reset_reason == "TARGET_HARD_TIMEOUT"
                else "TARGET_TRACKING"
            )
        elif age <= self.dropout_grace_sec:
            target_state = "TARGET_STABLE" if self.stable_latched else "TARGET_TRACKING"
        elif age <= self.soft_timeout_sec:
            target_state = "TARGET_TEMPORARILY_STALE"
        elif age <= self.hard_timeout_sec:
            target_state = "WAIT_TARGET_RECOVERY"
        else:
            target_state = "TARGET_LOST"
        if (self.reacquire_result == "OBSERVE_SPATIAL_DRIFT"
                and not self.stable_latched and age is not None):
            target_state = "TARGET_OBSERVING_DRIFT"
        return {
            "vision_sample_rate_hz": (sample_count - 1) / span if span > 0.0 else 0.0,
            "vision_valid_samples": sample_count,
            "spatial_jitter_mm": self._jitter_m * 1000.0,
            "spatial_drift_mm": self._drift_m * 1000.0,
            "target_jitter_mm": self._jitter_m * 1000.0,
            "target_motion_mm": (
                self.reacquire_distance_m * 1000.0
                if self.reacquire_distance_m is not None else self._drift_m * 1000.0
            ),
            "target_age_ms": age * 1000.0 if age is not None else None,
            "stable_duration_ms": (
                max(0.0, now - self._stable_since) * 1000.0
                if self._stable_since is not None else 0.0
            ),
            "stable_required_ms": 0.0,
            "target_stable": self.stable_latched,
            "acquisition_state": self.acquisition_state,
            "window_oldest_age_ms": (
                (now - recent[0][0]) * 1000.0 if recent else None
            ),
            "window_valid_samples": sample_count,
            "window_center_xyz": self._center_m,
            "jitter_p95_mm": self._jitter_m * 1000.0,
            "oldest_sample_distance_mm": self._oldest_distance_m * 1000.0,
            "stable_reset_count": self.reset_count,
            "last_stable_reset_reason": self.last_reset_reason,
            "stable_reset_reason": self.last_reset_reason,
            "last_valid_target_age_ms": age * 1000.0 if age is not None else None,
            "dropout_grace_ms": self.dropout_grace_sec * 1000.0,
            "consecutive_missing_frames": self.consecutive_missing_frames,
            "missing_frames_est": (
                max(self.consecutive_missing_frames, round(age * self._last_sample_rate_hz))
                if age is not None else 0
            ),
            "temporary_stale": target_state in (
                "TARGET_TEMPORARILY_STALE", "WAIT_TARGET_RECOVERY"
            ),
            "stable_latched": self.stable_latched,
            "frozen_target": self.frozen_region.point_m if self.frozen_region else None,
            "reacquire_distance_mm": (
                self.reacquire_distance_m * 1000.0
                if self.reacquire_distance_m is not None else None
            ),
            "reacquire_result": self.reacquire_result,
            "target_tracking_state": target_state,
            "gap_action": (
                "KEEP_STABLE" if self.stable_latched and age is not None
                and age <= self.dropout_grace_sec else
                "WAIT_FRESH_MEASUREMENT" if self.stable_latched else
                "TRACK_NEW_TARGET"
            ),
        }


def point_distance(first: tuple[float, float, float], second: tuple[float, float, float]) -> float:
    """Small dependency-free helper used by tests and diagnostics."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))
