"""
Visualize the cargo_planner algorithm with ROS 2.

This node is not the production planner. It replays the main planning steps
to generate PNG images that explain the algorithm:

- input occupancy grid
- ordered pallet list
- eroded free space
- valid anchors for 0 and 90 degrees
- selected anchor
- updated map after placing each pallet

The pallet list can be loaded from a YAML file with the same structure used by
the cargo_planner_msgs/CargoListRegistration service request:

    cargo_units:
      - id: pallet_01
        lx: 1.2
        ly: 0.8
        lz: 2.38
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from math import ceil, isfinite
from numbers import Real
from pathlib import Path
from typing import Iterable, Sequence

from nav_msgs.msg import OccupancyGrid
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
from PIL import Image, ImageDraw, ImageFont
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
import yaml


@dataclass(frozen=True)
class CargoUnitSpec:
    id: str  # noqa: A003 - Keep the same field name as cargo_planner_msgs/CargoUnit.
    lx: float
    ly: float
    lz: float

    @property
    def volume(self) -> float:
        return self.lx * self.ly * self.lz


PALETTE = [
    (54, 162, 235),
    (255, 159, 64),
    (75, 192, 192),
    (153, 102, 255),
    (255, 99, 132),
    (46, 204, 113),
    (241, 196, 15),
    (149, 165, 166),
]


def load_cargo_units(path: Path) -> list[CargoUnitSpec]:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if isinstance(data, dict):
        raw_units = data.get('cargo_units', [])
    elif isinstance(data, list):
        raw_units = data
    else:
        raise ValueError(f"Unsupported YAML format in '{path}'.")

    units: list[CargoUnitSpec] = []
    cargo_ids: set[str] = set()
    for idx, item in enumerate(raw_units, start=1):
        if not isinstance(item, dict):
            raise ValueError(f'cargo_units[{idx}] must be a mapping.')

        expected_keys = {'id', 'lx', 'ly', 'lz'}
        unknown_keys = sorted(set(item) - expected_keys)
        if unknown_keys:
            raise ValueError(f'cargo_units[{idx}] has unknown keys: {unknown_keys}.')

        missing_keys = sorted(expected_keys - set(item))
        if missing_keys:
            raise ValueError(f'cargo_units[{idx}] is missing fields: {missing_keys}.')

        unit_id = item['id']
        if not isinstance(unit_id, str) or not unit_id.strip():
            raise ValueError(f'cargo_units[{idx}].id must be a non-blank string.')
        if unit_id in cargo_ids:
            raise ValueError(f"cargo_units[{idx}] duplicates cargo ID '{unit_id}'.")
        cargo_ids.add(unit_id)

        dimensions: list[float] = []
        for key in ('lx', 'ly', 'lz'):
            raw_value = item[key]
            if not isinstance(raw_value, Real) or isinstance(raw_value, bool):
                raise ValueError(f'cargo_units[{idx}].{key} must be numeric.')
            value = float(raw_value)
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f'cargo_units[{idx}].{key} must be positive and finite.')
            dimensions.append(value)

        lx, ly, lz = dimensions
        units.append(CargoUnitSpec(unit_id, lx, ly, lz))

    if not units:
        raise ValueError(f"No cargo units found in '{path}'.")

    return units


def sort_units_by_volume(units: Sequence[CargoUnitSpec]) -> list[CargoUnitSpec]:
    return sorted(units, key=lambda unit: unit.volume, reverse=True)


def to_cells(metres: float, resolution: float) -> int:
    if metres == 0.0:
        return 0
    if not isfinite(metres) or not isfinite(resolution) or metres < 0.0 or resolution <= 0.0:
        raise ValueError('metres and resolution must be finite and valid.')
    return max(1, int(ceil(metres / resolution)))


def occupancy_grid_to_free_mask(grid: OccupancyGrid, occupied_threshold: int) -> np.ndarray:
    if occupied_threshold < 0 or occupied_threshold > 100:
        raise ValueError('occupied_threshold must be in [0, 100].')
    width = int(grid.info.width)
    height = int(grid.info.height)
    if width <= 0 or height <= 0:
        raise ValueError('OccupancyGrid has invalid dimensions.')
    if len(grid.data) != width * height:
        raise ValueError('OccupancyGrid data size does not match width * height.')

    cells = np.asarray(grid.data, dtype=np.int16).reshape((height, width))
    return (cells >= 0) & (cells < occupied_threshold)


def erode_rect_top_left(mask: np.ndarray, krows: int, kcols: int) -> np.ndarray:
    if krows <= 0 or kcols <= 0:
        return np.zeros_like(mask, dtype=bool)
    if mask.ndim != 2:
        raise ValueError('mask must be 2D.')

    padded = np.pad(
        mask.astype(np.uint8), ((0, krows - 1), (0, kcols - 1)), mode='constant', constant_values=0
    )
    windows = sliding_window_view(padded, (krows, kcols))
    return windows.all(axis=(-1, -2))


def erode_rect_centered(mask: np.ndarray, krows: int, kcols: int) -> np.ndarray:
    if krows <= 0 or kcols <= 0:
        return np.zeros_like(mask, dtype=bool)
    if mask.ndim != 2:
        raise ValueError('mask must be 2D.')

    pad_top = krows // 2
    pad_bottom = krows - pad_top - 1
    pad_left = kcols // 2
    pad_right = kcols - pad_left - 1
    padded = np.pad(
        mask.astype(np.uint8),
        ((pad_top, pad_bottom), (pad_left, pad_right)),
        mode='constant',
        constant_values=0,
    )
    windows = sliding_window_view(padded, (krows, kcols))
    return windows.all(axis=(-1, -2))


def select_anchor(valid_mask: np.ndarray, prefer_right_wall: bool) -> tuple[int, int] | None:
    rows, cols = valid_mask.shape
    col_range = range(cols - 1, -1, -1)

    for col in col_range:
        if prefer_right_wall:
            row_range = range(rows)
        else:
            row_range = range(rows - 1, -1, -1)
        for row in row_range:
            if valid_mask[row, col]:
                return row, col
    return None


def mark_occupied(
    work_map: np.ndarray,
    anchor_row: int,
    anchor_col: int,
    krows: int,
    kcols: int,
    margin_cells: int,
) -> None:
    y0 = max(0, anchor_row - margin_cells)
    x0 = max(0, anchor_col - margin_cells)
    y1 = min(work_map.shape[0], anchor_row + krows + margin_cells)
    x1 = min(work_map.shape[1], anchor_col + kcols + margin_cells)
    work_map[y0:y1, x0:x1] = False


def make_canvas(mask: np.ndarray) -> np.ndarray:
    canvas = np.empty((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    canvas[mask] = (255, 255, 255)
    canvas[~mask] = (0, 0, 0)
    return canvas


def paint_cells(canvas: np.ndarray, mask: np.ndarray, color: tuple[int, int, int]) -> None:
    canvas[mask] = color


def paint_rectangle(
    canvas: np.ndarray,
    anchor_row: int,
    anchor_col: int,
    krows: int,
    kcols: int,
    *,
    fill: tuple[int, int, int] | None = None,
    outline: tuple[int, int, int] | None = None,
) -> None:
    rows, cols = canvas.shape[:2]
    y0 = max(0, anchor_row)
    x0 = max(0, anchor_col)
    y1 = min(rows, anchor_row + krows)
    x1 = min(cols, anchor_col + kcols)

    if y0 >= y1 or x0 >= x1:
        return

    if fill is not None:
        canvas[y0:y1, x0:x1] = fill
    if outline is not None:
        canvas[y0 : y0 + 1, x0:x1] = outline
        canvas[y1 - 1 : y1, x0:x1] = outline
        canvas[y0:y1, x0 : x0 + 1] = outline
        canvas[y0:y1, x1 - 1 : x1] = outline


def paint_point(canvas: np.ndarray, row: int, col: int, color: tuple[int, int, int]) -> None:
    if 0 <= row < canvas.shape[0] and 0 <= col < canvas.shape[1]:
        canvas[row, col] = color


def scale_image(canvas: np.ndarray, scale: int) -> Image.Image:
    scaled = np.repeat(np.repeat(canvas, scale, axis=0), scale, axis=1)
    return Image.fromarray(scaled, mode='RGB')


def save_grid_png(
    mask: np.ndarray,
    path: Path,
    *,
    scale: int = 8,
    cell_masks: Iterable[tuple[np.ndarray, tuple[int, int, int]]] = (),
    rectangles: Iterable[
        tuple[int, int, int, int, tuple[int, int, int] | None, tuple[int, int, int] | None]
    ] = (),
    points: Iterable[tuple[int, int, tuple[int, int, int]]] = (),
) -> None:
    canvas = make_canvas(mask)
    for cell_mask, color in cell_masks:
        paint_cells(canvas, cell_mask, color)
    for anchor_row, anchor_col, krows, kcols, fill, outline in rectangles:
        paint_rectangle(canvas, anchor_row, anchor_col, krows, kcols, fill=fill, outline=outline)
    for row, col, color in points:
        paint_point(canvas, row, col, color)

    image = scale_image(canvas, scale)
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def save_pallet_order_png(units: Sequence[CargoUnitSpec], path: Path) -> None:
    rows = len(units) + 1
    width = 860
    row_h = 26
    header_h = 28
    image = Image.new('RGB', (width, header_h + rows * row_h), 'white')
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    columns = [('Orden', 10), ('ID', 90), ('lx', 260), ('ly', 390), ('lz', 520), ('Volumen', 650)]
    for label, x in columns:
        draw.text((x, 8), label, fill=(0, 0, 0), font=font)

    draw.line((10, 26, width - 10, 26), fill=(0, 0, 0), width=1)
    for idx, unit in enumerate(units, start=1):
        y = 26 + (idx - 1) * row_h + 5
        values = [
            (str(idx), 10),
            (unit.id, 90),
            (f'{unit.lx:.2f}', 260),
            (f'{unit.ly:.2f}', 390),
            (f'{unit.lz:.2f}', 520),
            (f'{unit.volume:.3f}', 650),
        ]
        for value, x in values:
            draw.text((x, y), value, fill=(0, 0, 0), font=font)
        if idx < len(units):
            draw.line(
                (10, 26 + idx * row_h, width - 10, 26 + idx * row_h), fill=(230, 230, 230), width=1
            )

    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def record_image(image_manifest: list[tuple[str, str]], path: Path, description: str) -> None:
    image_manifest.append((path.name, description))


def save_and_record_grid_png(
    mask: np.ndarray,
    path: Path,
    image_manifest: list[tuple[str, str]],
    description: str,
    *,
    scale: int,
    cell_masks: Iterable[tuple[np.ndarray, tuple[int, int, int]]] = (),
    rectangles: Iterable[
        tuple[int, int, int, int, tuple[int, int, int] | None, tuple[int, int, int] | None]
    ] = (),
    points: Iterable[tuple[int, int, tuple[int, int, int]]] = (),
) -> None:
    save_grid_png(
        mask, path, scale=scale, cell_masks=cell_masks, rectangles=rectangles, points=points
    )
    record_image(image_manifest, path, description)


@dataclass(frozen=True)
class CargoPlannerVisualDebuggerConfig:
    """Configuration for generating visual debugging artifacts."""

    topic: str
    yaml_file: Path | None
    output_dir: Path
    container_height: float
    pallet_count: int
    pallet_length: float
    pallet_width: float
    pallet_height: float
    occupied_threshold: int
    pallet_margin: float
    enable_rotation: bool
    image_scale: int


class CargoPlannerVisualDebugger:
    """Generate PNGs that explain the cargo_planner placement logic step by step."""

    def __init__(self, config: CargoPlannerVisualDebuggerConfig) -> None:
        """Store configuration, create a run directory, and load cargo units."""
        self._config = config
        self._run_dir = config.output_dir / datetime.now().strftime('%Y%m%d_%H%M%S')
        self._run_dir.mkdir(parents=True, exist_ok=True)
        self._cargo_units = self._load_units()

    @property
    def run_dir(self) -> Path:
        """Return the directory where this run writes its visual artifacts."""
        return self._run_dir

    def process_grid(self, msg: OccupancyGrid) -> None:
        """Generate all visual debugging artifacts for one occupancy grid."""
        resolution = float(msg.info.resolution)
        free_mask = occupancy_grid_to_free_mask(msg, self._config.occupied_threshold)
        image_manifest: list[tuple[str, str]] = []

        save_and_record_grid_png(
            free_mask,
            self._run_dir / '00_input_occupancy_grid.png',
            image_manifest,
            'Mapa inicial recibido',
            scale=self._config.image_scale,
        )

        ordered_units = sort_units_by_volume(self._cargo_units)
        save_pallet_order_png(ordered_units, self._run_dir / '01_sorted_pallet_list.png')
        record_image(
            image_manifest,
            self._run_dir / '01_sorted_pallet_list.png',
            'Lista de pallets ordenada por volumen descendente',
        )

        work_map = free_mask.copy()
        margin_cells = to_cells(self._config.pallet_margin, resolution)
        if margin_cells > 0:
            work_map = erode_rect_centered(work_map, 2 * margin_cells + 1, 2 * margin_cells + 1)
        save_and_record_grid_png(
            work_map,
            self._run_dir / '02_margin_eroded_free_space.png',
            image_manifest,
            'Mapa libre tras aplicar margen de seguridad',
            scale=self._config.image_scale,
        )

        placements: list[dict[str, object]] = []
        placed_count = 0

        for idx, unit in enumerate(ordered_units):
            step_tag = f'{idx + 1:02d}'
            kcols_0 = to_cells(unit.lx, resolution)
            krows_0 = to_cells(unit.ly, resolution)
            prefer_right = (placed_count % 2) == 0

            valid_0 = erode_rect_top_left(work_map, krows_0, kcols_0)
            anchor_0 = select_anchor(valid_0, prefer_right)
            save_and_record_grid_png(
                work_map,
                self._run_dir / f'step_{step_tag}_valid_anchors_0deg.png',
                image_manifest,
                f'Posiciones validas para {unit.id}, orientacion 0 grados',
                scale=self._config.image_scale,
                cell_masks=[(valid_0, (46, 204, 113))],
            )

            valid_90 = None
            anchor_90 = None
            kcols_90 = 0
            krows_90 = 0
            if self._config.enable_rotation:
                kcols_90 = to_cells(unit.ly, resolution)
                krows_90 = to_cells(unit.lx, resolution)
                valid_90 = erode_rect_top_left(work_map, krows_90, kcols_90)
                anchor_90 = select_anchor(valid_90, prefer_right)
                save_and_record_grid_png(
                    work_map,
                    self._run_dir / f'step_{step_tag}_valid_anchors_90deg.png',
                    image_manifest,
                    f'Posiciones validas para {unit.id}, orientacion 90 grados',
                    scale=self._config.image_scale,
                    cell_masks=[(valid_90, (46, 204, 113))],
                )

            if anchor_0 is None and anchor_90 is None:
                placements.append({'id': unit.id, 'placed': False})
                self._write_unplaced_snapshot(image_manifest, unit, step_tag, work_map)
                continue

            use_90 = False
            anchor = anchor_0
            krows = krows_0
            kcols = kcols_0
            eff_lx = unit.lx
            eff_ly = unit.ly
            if anchor_0 is not None and anchor_90 is not None:
                use_90 = anchor_90[1] > anchor_0[1]
                if use_90:
                    anchor = anchor_90
                    krows = krows_90
                    kcols = kcols_90
                    eff_lx = unit.ly
                    eff_ly = unit.lx
            elif anchor_90 is not None:
                use_90 = True
                anchor = anchor_90
                krows = krows_90
                kcols = kcols_90
                eff_lx = unit.ly
                eff_ly = unit.lx

            assert anchor is not None
            row, col = anchor
            selected_valid = valid_90 if use_90 and valid_90 is not None else valid_0
            selected_desc = '90 grados' if use_90 else '0 grados'

            save_and_record_grid_png(
                work_map,
                self._run_dir / f'step_{step_tag}_selected_anchor.png',
                image_manifest,
                f'Anclaje seleccionado para {unit.id} en orientacion {selected_desc}',
                scale=self._config.image_scale,
                cell_masks=[(selected_valid, (46, 204, 113))],
                rectangles=[(row, col, krows, kcols, (173, 216, 230), (52, 152, 219))],
                points=[(row, col, (231, 76, 60))],
            )

            mark_occupied(work_map, row, col, krows, kcols, margin_cells)
            color = PALETTE[placed_count % len(PALETTE)]
            placements.append(
                {
                    'id': unit.id,
                    'placed': True,
                    'row': row,
                    'col': col,
                    'krows': krows,
                    'kcols': kcols,
                    'rotated': use_90,
                    'eff_lx': eff_lx,
                    'eff_ly': eff_ly,
                }
            )
            placed_count += 1

            save_and_record_grid_png(
                work_map,
                self._run_dir / f'step_{step_tag}_after_place.png',
                image_manifest,
                f'Mapa despues de colocar {unit.id}',
                scale=self._config.image_scale,
                rectangles=[(row, col, krows, kcols, None, color)],
            )

        save_and_record_grid_png(
            work_map,
            self._run_dir / '07_final_plan.png',
            image_manifest,
            'Mapa final tras procesar todos los pallets',
            scale=self._config.image_scale,
        )
        self._write_image_manifest(image_manifest)
        self._write_manifest(msg, ordered_units, placements)

    def _load_units(self) -> list[CargoUnitSpec]:
        """Load cargo units from YAML or generate default pallet specs."""
        if self._config.yaml_file is not None:
            return load_cargo_units(self._config.yaml_file)

        if self._config.pallet_count <= 0:
            raise ValueError('pallet_count must be > 0 when pallets_yaml_file is empty.')
        return [
            CargoUnitSpec(
                f'pallet_{idx:02d}',
                self._config.pallet_length,
                self._config.pallet_width,
                self._config.pallet_height,
            )
            for idx in range(1, self._config.pallet_count + 1)
        ]

    def _write_manifest(
        self,
        msg: OccupancyGrid,
        ordered_units: Sequence[CargoUnitSpec],
        placements: Sequence[dict[str, object]],
    ) -> None:
        """Write a YAML manifest with inputs and placement results."""
        manifest = {
            'topic': self._config.topic,
            'pallets_yaml_file': (
                str(self._config.yaml_file) if self._config.yaml_file is not None else ''
            ),
            'container_height': self._config.container_height,
            'occupied_threshold': self._config.occupied_threshold,
            'pallet_margin': self._config.pallet_margin,
            'enable_rotation': self._config.enable_rotation,
            'resolution': float(msg.info.resolution),
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'ordered_units': [
                {'id': unit.id, 'lx': unit.lx, 'ly': unit.ly, 'lz': unit.lz, 'volume': unit.volume}
                for unit in ordered_units
            ],
            'placements': list(placements),
        }
        (self._run_dir / 'manifest.yaml').write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8'
        )

    def _write_unplaced_snapshot(
        self,
        image_manifest: list[tuple[str, str]],
        unit: CargoUnitSpec,
        step_tag: str,
        work_map: np.ndarray,
    ) -> None:
        """Write the placeholder images for a cargo unit that cannot be placed."""
        save_and_record_grid_png(
            work_map,
            self._run_dir / f'step_{step_tag}_selected_anchor.png',
            image_manifest,
            f'Sin anclaje valido para {unit.id}',
            scale=self._config.image_scale,
        )
        save_and_record_grid_png(
            work_map,
            self._run_dir / f'step_{step_tag}_after_place.png',
            image_manifest,
            f'Sin colocacion para {unit.id}',
            scale=self._config.image_scale,
        )

    def _write_image_manifest(self, image_manifest: Sequence[tuple[str, str]]) -> None:
        """Write the tab-separated image manifest."""
        lines = ['filename\tdescription']
        for filename, description in image_manifest:
            lines.append(f'{filename}\t{description}')
        (self._run_dir / 'image_manifest.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')


class CargoPlannerVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('cargo_planner_visualizer')
        self.declare_parameter('occupancy_grid_topic', '/container_occupancy_grid')
        self.declare_parameter('pallets_yaml_file', '')
        self.declare_parameter('output_dir', '/tmp/cargo_planner_visualizer')
        self.declare_parameter('container_height', 2.38)
        self.declare_parameter('pallet_count', 8)
        self.declare_parameter('pallet_length', 1.2)
        self.declare_parameter('pallet_width', 0.8)
        self.declare_parameter('pallet_height', 2.38)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('pallet_margin', 0.05)
        self.declare_parameter('enable_rotation', True)
        self.declare_parameter('one_shot', True)
        self.declare_parameter('image_scale', 8)

        self._topic = self.get_parameter('occupancy_grid_topic').value
        yaml_value = str(self.get_parameter('pallets_yaml_file').value).strip()
        self._yaml_file = Path(yaml_value) if yaml_value else None
        self._output_dir = Path(self.get_parameter('output_dir').value)
        self._container_height = float(self.get_parameter('container_height').value)
        self._pallet_count = int(self.get_parameter('pallet_count').value)
        self._pallet_length = float(self.get_parameter('pallet_length').value)
        self._pallet_width = float(self.get_parameter('pallet_width').value)
        self._pallet_height = float(self.get_parameter('pallet_height').value)
        self._occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self._pallet_margin = float(self.get_parameter('pallet_margin').value)
        self._enable_rotation = bool(self.get_parameter('enable_rotation').value)
        self._one_shot = bool(self.get_parameter('one_shot').value)
        self._image_scale = int(self.get_parameter('image_scale').value)
        self._processed = False
        self._run_dir = self._output_dir / datetime.now().strftime('%Y%m%d_%H%M%S')
        self._run_dir.mkdir(parents=True, exist_ok=True)

        self._cargo_units = self._load_units()
        self._subscription = self.create_subscription(
            OccupancyGrid,
            self._topic,
            self._on_grid,
            QoSProfile(depth=1),
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f"Waiting for OccupancyGrid on '{self._topic}'. Output dir: {self._run_dir}"
        )

    def _load_units(self) -> list[CargoUnitSpec]:
        if self._yaml_file is not None:
            return load_cargo_units(self._yaml_file)

        if self._pallet_count <= 0:
            raise ValueError('pallet_count must be > 0 when pallets_yaml_file is empty.')
        units = [
            CargoUnitSpec(
                f'pallet_{idx:02d}', self._pallet_length, self._pallet_width, self._pallet_height
            )
            for idx in range(1, self._pallet_count + 1)
        ]
        return units

    def _on_grid(self, msg: OccupancyGrid) -> None:
        if self._one_shot and self._processed:
            return
        self._processed = True

        try:
            self._process_grid(msg)
        except Exception as exc:  # pragma: no cover - runtime guard
            self.get_logger().error(f'Visualization failed: {exc}')
            raise

        if self._one_shot:
            self.get_logger().info('one_shot=true; shutting down.')
            rclpy.shutdown()

    def _process_grid(self, msg: OccupancyGrid) -> None:
        resolution = float(msg.info.resolution)
        free_mask = occupancy_grid_to_free_mask(msg, self._occupied_threshold)
        image_manifest: list[tuple[str, str]] = []

        save_and_record_grid_png(
            free_mask,
            self._run_dir / '00_input_occupancy_grid.png',
            image_manifest,
            'Mapa inicial recibido',
            scale=self._image_scale,
        )

        ordered_units = sort_units_by_volume(self._cargo_units)
        save_pallet_order_png(ordered_units, self._run_dir / '01_sorted_pallet_list.png')
        record_image(
            image_manifest,
            self._run_dir / '01_sorted_pallet_list.png',
            'Lista de pallets ordenada por volumen descendente',
        )

        work_map = free_mask.copy()
        margin_cells = to_cells(self._pallet_margin, resolution)
        if margin_cells > 0:
            work_map = erode_rect_centered(work_map, 2 * margin_cells + 1, 2 * margin_cells + 1)
        save_and_record_grid_png(
            work_map,
            self._run_dir / '02_margin_eroded_free_space.png',
            image_manifest,
            'Mapa libre tras aplicar margen de seguridad',
            scale=self._image_scale,
        )

        placements: list[dict[str, object]] = []
        placed_count = 0

        for idx, unit in enumerate(ordered_units):
            step_tag = f'{idx + 1:02d}'
            kcols_0 = to_cells(unit.lx, resolution)
            krows_0 = to_cells(unit.ly, resolution)
            prefer_right = (placed_count % 2) == 0

            valid_0 = erode_rect_top_left(work_map, krows_0, kcols_0)
            anchor_0 = select_anchor(valid_0, prefer_right)
            save_and_record_grid_png(
                work_map,
                self._run_dir / f'step_{step_tag}_valid_anchors_0deg.png',
                image_manifest,
                f'Posiciones válidas para {unit.id}, orientación 0 grados',
                scale=self._image_scale,
                cell_masks=[(valid_0, (46, 204, 113))],
            )
            valid_90 = None
            anchor_90 = None
            kcols_90 = 0
            krows_90 = 0

            if self._enable_rotation:
                kcols_90 = to_cells(unit.ly, resolution)
                krows_90 = to_cells(unit.lx, resolution)
                valid_90 = erode_rect_top_left(work_map, krows_90, kcols_90)
                anchor_90 = select_anchor(valid_90, prefer_right)
                save_and_record_grid_png(
                    work_map,
                    self._run_dir / f'step_{step_tag}_valid_anchors_90deg.png',
                    image_manifest,
                    f'Posiciones válidas para {unit.id}, orientación 90 grados',
                    scale=self._image_scale,
                    cell_masks=[(valid_90, (46, 204, 113))],
                )

            if anchor_0 is None and anchor_90 is None:
                placements.append({'id': unit.id, 'placed': False})
                self._write_unplaced_snapshot(image_manifest, unit, step_tag, work_map)
                continue

            use_90 = False
            anchor = anchor_0
            krows = krows_0
            kcols = kcols_0
            eff_lx = unit.lx
            eff_ly = unit.ly
            if anchor_0 is not None and anchor_90 is not None:
                use_90 = anchor_90[1] > anchor_0[1]
                if use_90:
                    anchor = anchor_90
                    krows = krows_90
                    kcols = kcols_90
                    eff_lx = unit.ly
                    eff_ly = unit.lx
            elif anchor_90 is not None:
                use_90 = True
                anchor = anchor_90
                krows = krows_90
                kcols = kcols_90
                eff_lx = unit.ly
                eff_ly = unit.lx

            assert anchor is not None
            row, col = anchor
            selected_valid = valid_90 if use_90 and valid_90 is not None else valid_0
            selected_desc = '90 grados' if use_90 else '0 grados'

            save_and_record_grid_png(
                work_map,
                self._run_dir / f'step_{step_tag}_selected_anchor.png',
                image_manifest,
                f'Anclaje seleccionado para {unit.id} en orientación {selected_desc}',
                scale=self._image_scale,
                cell_masks=[(selected_valid, (46, 204, 113))],
                rectangles=[(row, col, krows, kcols, (173, 216, 230), (52, 152, 219))],
                points=[(row, col, (231, 76, 60))],
            )

            mark_occupied(work_map, row, col, krows, kcols, margin_cells)
            color = PALETTE[placed_count % len(PALETTE)]
            placements.append(
                {
                    'id': unit.id,
                    'placed': True,
                    'row': row,
                    'col': col,
                    'krows': krows,
                    'kcols': kcols,
                    'rotated': use_90,
                    'eff_lx': eff_lx,
                    'eff_ly': eff_ly,
                }
            )
            placed_count += 1

            save_and_record_grid_png(
                work_map,
                self._run_dir / f'step_{step_tag}_after_place.png',
                image_manifest,
                f'Mapa después de colocar {unit.id}',
                scale=self._image_scale,
                rectangles=[(row, col, krows, kcols, None, color)],
            )

        save_and_record_grid_png(
            work_map,
            self._run_dir / '07_final_plan.png',
            image_manifest,
            'Mapa final tras procesar todos los pallets',
            scale=self._image_scale,
        )
        self._write_image_manifest(image_manifest)
        self._write_manifest(msg, ordered_units, placements)
        self.get_logger().info(f'Images written to {self._run_dir}')

    def _write_manifest(
        self,
        msg: OccupancyGrid,
        ordered_units: Sequence[CargoUnitSpec],
        placements: Sequence[dict[str, object]],
    ) -> None:
        manifest = {
            'topic': self._topic,
            'pallets_yaml_file': str(self._yaml_file) if self._yaml_file is not None else '',
            'container_height': self._container_height,
            'occupied_threshold': self._occupied_threshold,
            'pallet_margin': self._pallet_margin,
            'enable_rotation': self._enable_rotation,
            'resolution': float(msg.info.resolution),
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'ordered_units': [
                {'id': unit.id, 'lx': unit.lx, 'ly': unit.ly, 'lz': unit.lz, 'volume': unit.volume}
                for unit in ordered_units
            ],
            'placements': list(placements),
        }
        (self._run_dir / 'manifest.yaml').write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8'
        )

    def _write_unplaced_snapshot(
        self,
        image_manifest: list[tuple[str, str]],
        unit: CargoUnitSpec,
        step_tag: str,
        work_map: np.ndarray,
    ) -> None:
        save_and_record_grid_png(
            work_map,
            self._run_dir / f'step_{step_tag}_selected_anchor.png',
            image_manifest,
            f'Sin anclaje válido para {unit.id}',
            scale=self._image_scale,
        )
        save_and_record_grid_png(
            work_map,
            self._run_dir / f'step_{step_tag}_after_place.png',
            image_manifest,
            f'Sin colocación para {unit.id}',
            scale=self._image_scale,
        )

    def _write_image_manifest(self, image_manifest: Sequence[tuple[str, str]]) -> None:
        lines = ['filename\tdescription']
        for filename, description in image_manifest:
            lines.append(f'{filename}\t{description}')
        (self._run_dir / 'image_manifest.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CargoPlannerVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
