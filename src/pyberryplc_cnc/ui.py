"""
NiceGUI editor for compiling XYZ CNC paths.

The UI lets a user define path vertices and motion timing while reusing the
same motor configuration TOML file as ``XYZMotionController``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from nicegui import ui

from .trajectory import compile_xyz_path


@dataclass
class _VertexControls:
    """
    UI controls belonging to one editable XYZ vertex row.

    Parameters
    ----------
    container:
        NiceGUI row element containing the controls.
    x:
        Numeric input for the X-coordinate.
    y:
        Numeric input for the Y-coordinate.
    z:
        Numeric input for the Z-coordinate.
    """

    container: object
    x: object
    y: object
    z: object


class CNCPathEditor:
    """
    Browser editor for compiling XYZ paths to stepper trajectory JSON.

    Parameters
    ----------
    port:
        NiceGUI server port.
    title:
        Browser page title.
    """

    def __init__(
        self,
        port: int = 8082,
        title: str = "pyberryplc CNC",
    ) -> None:
        """
        Initialize the editor state.

        Parameters
        ----------
        port:
            NiceGUI server port.
        title:
            Browser page title.
        """
        self.port = port
        self.title = title
        self._vertex_rows: list[_VertexControls] = []
        self._vertices_column = None
        self._config_path_input = None
        self._output_path_input = None
        self._feed_rate_input = None
        self._blend_time_input = None
        self._status_label = None

    def build(self) -> None:
        """
        Build the NiceGUI controls.
        """
        ui.page_title(self.title)

        with ui.header().classes("items-center justify-between"):
            ui.label("pyberryplc CNC").classes("text-lg font-semibold")
            ui.button(icon="save", on_click=self._compile).props("flat round")

        with ui.row().classes("w-full items-start gap-6 p-4"):
            with ui.column().classes("w-96 gap-3"):
                self._config_path_input = ui.input(
                    "motor_config.toml",
                    value="motor_config.toml",
                ).props("outlined dense").classes("w-full")
                self._output_path_input = ui.input(
                    "Output JSON",
                    value="trajectory.json",
                ).props("outlined dense").classes("w-full")
                with ui.row().classes("w-full gap-3"):
                    self._feed_rate_input = ui.number(
                        "Feed rate",
                        value=10.0,
                        min=0.0,
                        step=1.0,
                    ).props("outlined dense").classes("flex-1")
                    self._blend_time_input = ui.number(
                        "Blend time",
                        value=0.0,
                        min=0.0,
                        step=0.05,
                    ).props("outlined dense").classes("flex-1")
                with ui.row().classes("gap-2"):
                    ui.button(icon="add", on_click=self._add_vertex).props(
                        "outline round"
                    )
                    ui.button(icon="save", on_click=self._compile).props(
                        "color=primary"
                    )
                self._status_label = ui.label("").classes("text-sm")

            with ui.column().classes("flex-1 gap-2"):
                with ui.row().classes("w-full items-center justify-between"):
                    ui.label("Vertices").classes("text-base font-medium")
                self._vertices_column = ui.column().classes("w-full gap-2")

        self._add_vertex(0.0, 0.0, 0.0)
        self._add_vertex(10.0, 0.0, 0.0)

    def run(self) -> None:
        """
        Build and run the NiceGUI application.
        """
        self.build()
        ui.run(title=self.title, port=self.port)

    def _add_vertex(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
    ) -> None:
        """
        Add one editable vertex row.

        Parameters
        ----------
        x:
            Initial X-coordinate.
        y:
            Initial Y-coordinate.
        z:
            Initial Z-coordinate.
        """
        if self._vertices_column is None:
            return

        with self._vertices_column:
            row_container = ui.row().classes("w-full items-center gap-2")
            with row_container:
                x_input = ui.number("X", value=x, step=1.0).props(
                    "outlined dense"
                ).classes("w-28")
                y_input = ui.number("Y", value=y, step=1.0).props(
                    "outlined dense"
                ).classes("w-28")
                z_input = ui.number("Z", value=z, step=1.0).props(
                    "outlined dense"
                ).classes("w-28")
                row = _VertexControls(
                    container=row_container,
                    x=x_input,
                    y=y_input,
                    z=z_input,
                )
                ui.button(
                    icon="delete",
                    on_click=lambda row_=row: self._remove_vertex(row_),
                ).props("flat round")

        self._vertex_rows.append(row)

    def _remove_vertex(self, row: _VertexControls) -> None:
        """
        Remove one editable vertex row.

        Parameters
        ----------
        row:
            Vertex row to remove.
        """
        if len(self._vertex_rows) <= 2:
            ui.notify("At least two vertices are required.", color="warning")
            return
        self._vertex_rows.remove(row)
        # noinspection PyUnresolvedReferences
        row.container.delete()

    # noinspection PyUnresolvedReferences
    def _vertices(self) -> list[tuple[float, float, float]]:
        """
        Return the currently entered vertices.

        Returns
        -------
        list[tuple[float, float, float]]
            XYZ vertices from the UI.
        """
        return [
            (
                float(row.x.value),
                float(row.y.value),
                float(row.z.value),
            )
            for row in self._vertex_rows
        ]

    def _compile(self) -> None:
        """
        Compile the entered path and save it as JSON.
        """
        try:
            config_path = Path(str(self._config_path_input.value))
            output_path = Path(str(self._output_path_input.value))
            feed_rate = float(self._feed_rate_input.value)
            blend_time = float(self._blend_time_input.value)
            compiled = compile_xyz_path(
                vertices=self._vertices(),  #type: ignore
                motor_config_filepath=config_path,
                feed_rate=feed_rate,
                dt_blends=blend_time,
            )
            compiled.save(output_path)
        except Exception as exc:
            ui.notify(str(exc), color="negative")
            if self._status_label is not None:
                self._status_label.set_text("Compilation failed.")
            return

        num_segments = len(compiled.stepper_trajectory)
        ui.notify(f"Saved {num_segments} segments.", color="positive")
        if self._status_label is not None:
            self._status_label.set_text(
                f"Saved {num_segments} segments to {output_path}."
            )


def main() -> None:
    """
    Start the CNC path editor.
    """
    editor = CNCPathEditor()
    editor.run()


if __name__ in {"__main__", "__mp_main__"}:
    main()
