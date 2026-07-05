from dataclasses import dataclass
import math

from .ros_utils import make_inspection_point


@dataclass(frozen=True)
class InspectionRegion:
    name: str
    min_x: float
    min_y: float
    max_x: float
    max_y: float


@dataclass(frozen=True)
class RegionGenerationResult:
    points: list
    first_error: str | None
    warnings: list


def generate_region_points(regions, sweep_spacing, region_margin):
    generated = []
    warnings = []
    first_error = None
    for region in regions:
        region_points = generate_points_for_region(region, sweep_spacing, region_margin)
        for point_index, (x, y) in enumerate(region_points, start=1):
            generated.append(
                make_inspection_point(
                    f"{region.name}_P{point_index}",
                    x,
                    y,
                    0.0,
                )
            )
        if not region_points:
            message = (
                f"{region.name} is too small for spacing={sweep_spacing:.2f}m "
                f"and margin={region_margin:.2f}m"
            )
            first_error = first_error or message
            warnings.append(message)

    assign_path_headings(generated)
    return RegionGenerationResult(generated, first_error, warnings)


def generate_points_for_region(region, sweep_spacing, region_margin):
    min_x = region.min_x + region_margin
    min_y = region.min_y + region_margin
    max_x = region.max_x - region_margin
    max_y = region.max_y - region_margin
    if min_x > max_x or min_y > max_y:
        return []

    width = max_x - min_x
    height = max_y - min_y
    spacing = max(sweep_spacing, 0.05)

    points = []
    if width >= height:
        rows = sweep_positions(min_y, max_y, spacing)
        for row_index, y in enumerate(rows):
            if row_index % 2 == 0:
                points.append((min_x, y))
                if width > 0.02:
                    points.append((max_x, y))
            else:
                points.append((max_x, y))
                if width > 0.02:
                    points.append((min_x, y))
    else:
        columns = sweep_positions(min_x, max_x, spacing)
        for col_index, x in enumerate(columns):
            if col_index % 2 == 0:
                points.append((x, min_y))
                if height > 0.02:
                    points.append((x, max_y))
            else:
                points.append((x, max_y))
                if height > 0.02:
                    points.append((x, min_y))
    return points


def sweep_positions(start, end, spacing):
    if start > end:
        return []
    positions = []
    value = start
    while value <= end + 1e-9:
        positions.append(value)
        value += spacing
    if not positions or end - positions[-1] > min(spacing * 0.5, 0.10):
        positions.append(end)
    return positions


def assign_path_headings(points):
    for index, point in enumerate(points):
        target = None
        source = point
        if index + 1 < len(points):
            target = points[index + 1]
        elif index > 0:
            target = point
            source = points[index - 1]
        if target is None:
            continue
        dx = target.x - source.x
        dy = target.y - source.y
        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
            points[index].theta = math.atan2(dy, dx)


def regions_to_yaml_data(regions, sweep_spacing, region_margin):
    return {
        "version": 1,
        "map_frame": "map",
        "sweep_spacing": sweep_spacing,
        "region_margin": region_margin,
        "regions": [
            {
                "name": region.name,
                "min_x": region.min_x,
                "min_y": region.min_y,
                "max_x": region.max_x,
                "max_y": region.max_y,
            }
            for region in regions
        ],
    }


def regions_from_yaml(data):
    if int(data.get("version", 1)) != 1:
        raise ValueError("unsupported inspection region file version")
    if data.get("map_frame", "map") != "map":
        raise ValueError("inspection region file must use map_frame=map")

    regions = []
    for index, item in enumerate(data.get("regions", []), start=1):
        min_x = float(item["min_x"])
        min_y = float(item["min_y"])
        max_x = float(item["max_x"])
        max_y = float(item["max_y"])
        regions.append(
            InspectionRegion(
                name=str(item.get("name") or f"REGION_{index}"),
                min_x=min(min_x, max_x),
                min_y=min(min_y, max_y),
                max_x=max(min_x, max_x),
                max_y=max(min_y, max_y),
            )
        )
    return regions
