"""rqt plugin for G1 Inspire hand task control."""

from __future__ import annotations

import math
import sys
import threading
import types
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
from typing import Sequence

from ament_index_python.packages import get_package_prefix
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rqt_gui_py.plugin import Plugin
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from python_qt_binding.QtCore import QEvent, QObject, Qt, QTimer, Signal
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QShortcut,
    QSizePolicy,
    QSlider,
    QVBoxLayout,
    QWidget,
)


def _load_cli_controller():
    candidates = []
    try:
        candidates.append(
            Path(get_package_prefix('inspire_tutorials'))
            / 'lib'
            / 'inspire_tutorials'
            / 'g1_task_control_ros2'
        )
    except Exception:
        pass
    candidates.append(Path(__file__).resolve().parents[1] / 'scripts' / 'g1_task_control_ros2')

    for source in candidates:
        if source.is_file():
            loader = SourceFileLoader('g1_task_control_ros2_shared', str(source))
            spec = spec_from_loader(loader.name, loader)
            module = module_from_spec(spec) if spec is not None else types.ModuleType(loader.name)
            sys.modules[loader.name] = module
            loader.exec_module(module)
            return module
    raise RuntimeError('Could not locate g1_task_control_ros2')


_controller = _load_cli_controller()
HAND_JOINT_NAMES = _controller.HAND_JOINT_NAMES
CONTROLLER_NAMES = _controller.CONTROLLER_NAMES
TASK_TABLE = _controller.TASK_TABLE
OPEN_VALUES = _controller.OPEN_VALUES
CLOSE_VALUES = _controller.CLOSE_VALUES
format_command_values = _controller.format_command_values

MANUAL_JOINT_LABELS = ('Thumb yaw', 'Thumb pitch', 'Index', 'Middle', 'Ring', 'Pinky')
MANUAL_SLIDER_MIN = 0.0
MANUAL_SLIDER_MAX_VALUES = tuple(math.degrees(value) for value in (1.3, 0.6, 1.7, 1.7, 1.7, 1.7))
MANUAL_SLIDER_DURATION_SEC = 0.1
SLIDER_SCALE = 10
FOCUS_BLUE = '#1e88ff'


class _Signals(QObject):
    finished = Signal(str, object, bool)
    slider_finished = Signal(object, bool)


class G1TaskControlPlugin(Plugin):
    """RQT-native G1 Inspire hand task controller."""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('G1TaskControlPlugin')

        self._node = rclpy.create_node('g1_task_control_rqt')
        self._action_clients = {
            hand: ActionClient(
                self._node,
                FollowJointTrajectory,
                f'/{controller}/follow_joint_trajectory',
            )
            for hand, controller in CONTROLLER_NAMES.items()
        }
        self._trajectory_publishers = {
            hand: self._node.create_publisher(
                JointTrajectory,
                f'/{controller}/joint_trajectory',
                10,
            )
            for hand, controller in CONTROLLER_NAMES.items()
        }

        self._signals = _Signals()
        self._signals.finished.connect(self._finish_action)
        self._signals.slider_finished.connect(self._finish_manual_slider_send)

        self._current_task = None
        self._current_phase = 'none'
        self._selected_hand = None
        self._selected_task_number = None
        self._busy = False
        self._duration_sec = 1.0
        self._send_action_enabled = True
        self._send_topic_enabled = True
        self._manual_slider_worker_running = False
        self._manual_slider_pending = None
        self._manual_slider_updating = False

        self._buttons = []
        self._task_buttons = {}
        self._manual_sliders = []
        self._manual_value_labels = []

        self._widget = QWidget()
        self._widget.setObjectName('G1TaskControl')
        self._widget.setWindowTitle('G1 Inspire Hand Control')
        self._apply_style()
        self._build_ui()
        self._bind_shortcuts()
        self._refresh_state('Ready')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + f' ({context.serial_number()})'
            )
        context.add_widget(self._widget)

    def _apply_style(self) -> None:
        self._widget.setStyleSheet(
            f'''
            QPushButton:focus, QRadioButton:focus {{
                border: 2px solid {FOCUS_BLUE};
                border-radius: 4px;
                padding: 3px;
            }}
            QPushButton[selectedTask="true"] {{
                border: 2px solid {FOCUS_BLUE};
                border-radius: 4px;
                padding: 3px;
                background: #dcecff;
            }}
            QSlider[focused="true"]::handle:horizontal {{
                background: {FOCUS_BLUE};
                border: 1px solid #0b5fc0;
                width: 18px;
                margin: -5px 0;
                border-radius: 4px;
            }}
            QSlider::handle:horizontal {{
                width: 16px;
            }}
            '''
        )

    def _build_ui(self) -> None:
        root_layout = QVBoxLayout(self._widget)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        scroll.setWidget(content)
        root_layout.addWidget(scroll)

        title = QLabel('G1 Inspire Hand Control')
        font = title.font()
        font.setPointSize(font.pointSize() + 4)
        font.setBold(True)
        title.setFont(font)
        layout.addWidget(title)

        status_row = QHBoxLayout()
        self._task_label = QLabel('No task selected')
        self._phase_label = QLabel('Phase: none')
        self._status_label = QLabel('Ready')
        status_row.addWidget(self._task_label)
        status_row.addWidget(self._phase_label)
        status_row.addStretch(1)
        status_row.addWidget(self._status_label)
        layout.addLayout(status_row)

        hand_box = QGroupBox('Hand')
        hand_layout = QHBoxLayout(hand_box)
        self._hand_group = QButtonGroup(self._widget)
        for hand in ('left', 'right'):
            radio = QRadioButton(hand.capitalize())
            radio.setFocusPolicy(Qt.StrongFocus)
            radio.setChecked(hand == self.current_hand())
            radio.clicked.connect(lambda _checked=False, hand=hand: self.select_hand(hand))
            self._hand_group.addButton(radio)
            hand_layout.addWidget(radio)
        hand_layout.addSpacing(20)
        hand_layout.addWidget(QLabel('Duration (sec, 0.1-5.0)'))
        self._duration = QDoubleSpinBox()
        self._duration.setRange(0.1, 5.0)
        self._duration.setSingleStep(0.1)
        self._duration.setDecimals(2)
        self._duration.setValue(1.0)
        self._duration.lineEdit().returnPressed.connect(self.apply_duration_and_release_focus)
        hand_layout.addWidget(self._duration)
        hand_layout.addStretch(1)
        layout.addWidget(hand_box)

        output_box = QGroupBox('ROS Output')
        output_layout = QGridLayout(output_box)
        self._send_action = QCheckBox('FollowJointTrajectory Action')
        self._send_topic = QCheckBox('JointTrajectory Topic')
        self._send_action.setChecked(True)
        self._send_topic.setChecked(True)
        output_layout.addWidget(self._send_action, 0, 0)
        output_layout.addWidget(self._send_topic, 0, 1)
        output_layout.addWidget(QLabel('Action: /<hand>_hand_controller/follow_joint_trajectory'), 1, 0, 1, 2)
        output_layout.addWidget(QLabel('Topic: /<hand>_hand_controller/joint_trajectory'), 2, 0, 1, 2)
        layout.addWidget(output_box)

        tasks_box = QGroupBox('Tasks')
        tasks_layout = QGridLayout(tasks_box)
        for col in range(3):
            tasks_layout.setColumnStretch(col, 1)
        self._task_group = QButtonGroup(self._widget)
        self._task_group.setExclusive(True)
        for number, task in TASK_TABLE.items():
            row = (number - 1) // 3
            col = (number - 1) % 3
            button = QPushButton(f'{number}. {task.name}')
            button.setCheckable(True)
            button.setFocusPolicy(Qt.StrongFocus)
            button.clicked.connect(lambda _checked=False, number=number: self.select_task(number))
            self._task_group.addButton(button, number)
            self._task_buttons[number] = button
            self._buttons.append(button)
            tasks_layout.addWidget(button, row, col)
        layout.addWidget(tasks_box)

        manual_box = QGroupBox('Manual Angles (deg: thumb_yaw thumb_pitch index middle ring pinky)')
        manual_layout = QGridLayout(manual_box)
        manual_layout.setColumnStretch(1, 1)
        self._manual = QLineEdit()
        self._manual.returnPressed.connect(self.send_manual_action)
        manual_layout.addWidget(self._manual, 0, 0, 1, 2)
        manual_button = QPushButton('Send Manual')
        manual_button.setFocusPolicy(Qt.StrongFocus)
        manual_button.clicked.connect(self.send_manual_action)
        manual_layout.addWidget(manual_button, 0, 2)
        self._buttons.append(manual_button)

        for index, (label, max_value) in enumerate(zip(MANUAL_JOINT_LABELS, MANUAL_SLIDER_MAX_VALUES)):
            manual_layout.addWidget(QLabel(label), index + 1, 0)
            slider = QSlider(Qt.Horizontal)
            slider.setFocusPolicy(Qt.StrongFocus)
            slider.setRange(int(MANUAL_SLIDER_MIN * SLIDER_SCALE), int(round(max_value * SLIDER_SCALE)))
            slider.setSingleStep(1)
            slider.setPageStep(10)
            slider.setProperty('focused', False)
            slider.valueChanged.connect(lambda _value, index=index: self._on_manual_slider_change(index))
            slider.sliderPressed.connect(slider.setFocus)
            self._bind_slider_shortcuts(slider, index)
            self._manual_sliders.append(slider)
            manual_layout.addWidget(slider, index + 1, 1)
            value_label = QLabel('0.0')
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._manual_value_labels.append(value_label)
            manual_layout.addWidget(value_label, index + 1, 2)
        layout.addWidget(manual_box)

        commands_box = QGroupBox('Commands')
        commands_layout = QGridLayout(commands_box)
        for col in range(3):
            commands_layout.setColumnStretch(col, 1)
        self._add_action(commands_layout, 'Setting (s)', self.setting, 0, 0)
        self._add_action(commands_layout, 'Prepare / Start (g)', self.grasp, 0, 1)
        self._add_action(commands_layout, 'Open (o)', lambda: self.open_hand(self.current_hand()), 0, 2)
        self._add_action(commands_layout, 'Close (c)', lambda: self.close_hand(self.current_hand()), 1, 0)
        self._add_action(commands_layout, 'Close Both + Quit (q)', self.close_both_and_quit, 1, 1, 1, 2)
        layout.addWidget(commands_box)

        layout.addStretch(1)
        self._widget.setMinimumSize(720, 680)
        self._widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def _add_action(self, layout, text: str, action, row: int, col: int, row_span: int = 1, col_span: int = 1):
        button = QPushButton(text)
        button.setFocusPolicy(Qt.StrongFocus)
        button.clicked.connect(lambda _checked=False: self.run_action(text, action))
        layout.addWidget(button, row, col, row_span, col_span)
        self._buttons.append(button)

    def _bind_slider_shortcuts(self, slider: QSlider, index: int) -> None:
        slider.installEventFilter(self)
        for sequence, action in (
            ('Shift+Left', lambda index=index: self._adjust_manual_slider(index, -10)),
            ('Shift+Right', lambda index=index: self._adjust_manual_slider(index, 10)),
            ('Up', lambda index=index: self._focus_manual_slider(index - 1)),
            ('Down', lambda index=index: self._focus_manual_slider(index + 1)),
        ):
            shortcut = QShortcut(QKeySequence(sequence), slider)
            shortcut.setContext(Qt.WidgetShortcut)
            shortcut.activated.connect(action)

    def eventFilter(self, watched, event):
        if watched in self._manual_sliders:
            if event.type() == QEvent.FocusIn:
                watched.setProperty('focused', True)
                watched.style().unpolish(watched)
                watched.style().polish(watched)
            elif event.type() == QEvent.FocusOut:
                watched.setProperty('focused', False)
                watched.style().unpolish(watched)
                watched.style().polish(watched)
        return super().eventFilter(watched, event)

    def _bind_shortcuts(self) -> None:
        for key, action in (
            ('r', lambda: self.select_hand('right')),
            ('l', lambda: self.select_hand('left')),
            ('s', lambda: self.run_action('Setting', self.setting)),
            ('g', lambda: self.run_action('Prepare / Start', self.grasp)),
            ('o', lambda: self.run_action('Open', lambda: self.open_hand(self.current_hand()))),
            ('c', lambda: self.run_action('Close', lambda: self.close_hand(self.current_hand()))),
            ('q', lambda: self.run_action('Close Both + Quit', self.close_both_and_quit, close_after=True)),
        ):
            self._shortcut(key, action)
        for number in TASK_TABLE:
            self._shortcut(str(number), lambda number=number: self.select_task(number))

    def _shortcut(self, key: str, action) -> None:
        shortcut = QShortcut(QKeySequence(key), self._widget)
        shortcut.setContext(Qt.WidgetWithChildrenShortcut)
        shortcut.activated.connect(lambda action=action: self._run_shortcut(action))

    def _run_shortcut(self, action) -> None:
        if QApplication.focusWidget() in (self._duration.lineEdit(), self._manual):
            return
        action()

    def current_hand(self) -> str:
        if self._selected_hand:
            return self._selected_hand
        if self._current_task:
            return self._current_task.default_hand
        return 'left'

    def apply_duration(self) -> bool:
        duration = float(self._duration.value())
        if not 0.1 <= duration <= 5.0:
            QMessageBox.warning(self._widget, 'G1 Inspire Hand Control', 'Duration must be between 0.1 and 5.0 seconds.')
            self._duration.setValue(1.0)
            return False
        self._duration_sec = duration
        self._status_label.setText(f'Duration: {duration:.2f} sec')
        return True

    def _capture_output_options(self) -> None:
        self._send_action_enabled = self._send_action.isChecked()
        self._send_topic_enabled = self._send_topic.isChecked()

    def apply_duration_and_release_focus(self) -> None:
        if self.apply_duration():
            self._widget.setFocus()

    def select_hand(self, hand: str) -> None:
        if self._busy:
            return
        self._selected_hand = hand
        if self._current_task:
            self._current_phase = 'selected'
        self._sync_hand_buttons()
        self._refresh_state(f'Selected {hand} hand')
        if self._current_task:
            print(f'Current task: {self._current_task.name}. Press g for prepare-grasp or s for setting.')

    def select_task(self, number: int) -> None:
        if self._busy:
            return
        task = TASK_TABLE.get(number)
        if task is None:
            return
        self._current_task = task
        self._selected_task_number = number
        self._current_phase = 'selected'
        self._sync_hand_buttons()
        self._sync_task_buttons()
        self._refresh_state(f'Selected task {number}')
        print(f'Selected {number}: {task.name} ({self.current_hand()}). Press g for prepare-grasp or s for setting.')

    def _sync_hand_buttons(self) -> None:
        hand = self.current_hand()
        for button in self._hand_group.buttons():
            button.setChecked(button.text().lower() == hand)

    def _sync_task_buttons(self) -> None:
        for number, button in self._task_buttons.items():
            selected = number == self._selected_task_number
            button.setChecked(selected)
            button.setProperty('selectedTask', selected)
            button.style().unpolish(button)
            button.style().polish(button)

    def setting(self) -> None:
        self.send_task_pose('setting', 'setting')

    def grasp(self) -> None:
        if self._current_task is None:
            print('Select a task number first.')
            return
        if self._current_phase in ('selected', 'setting', 'manual', 'start-grasp'):
            self.send_task_pose('prepare', 'prepare-grasp')
        elif self._current_phase == 'prepare-grasp':
            self.send_task_pose('grasp', 'start-grasp')
        else:
            print('Press g after selecting a task, setting, or manual pose.')

    def send_task_pose(self, pose_key: str, phase_name: str) -> None:
        if self._current_task is None:
            print('Select a task number first.')
            return
        values = getattr(self._current_task, pose_key)
        hand = self.current_hand()
        if self._send_values(hand, values, phase_name):
            self._current_phase = phase_name
            print(f'{phase_name}: {self._current_task.name} ({hand})')
            print(format_command_values(values))

    def open_hand(self, hand: str) -> None:
        if self._send_values(hand, OPEN_VALUES, 'open'):
            self._current_phase = 'open'
            print(f'open: {hand}')
            print(format_command_values(OPEN_VALUES))

    def close_hand(self, hand: str) -> None:
        if self._send_values(hand, CLOSE_VALUES, 'close'):
            self._current_phase = 'close'
            print(f'close: {hand}')
            print(format_command_values(CLOSE_VALUES))

    def close_both_hands(self) -> None:
        ok = True
        for hand in ('right', 'left'):
            ok = self._send_values(hand, CLOSE_VALUES, 'close') and ok
        if ok:
            self._current_phase = 'close'
            print('close: both')
            print(format_command_values(CLOSE_VALUES))

    def parse_manual_values(self):
        raw = self._manual.text().strip().replace(',', ' ').split()
        if len(raw) != 6:
            QMessageBox.warning(
                self._widget,
                'G1 Inspire Hand Control',
                'Enter exactly six degree values: thumb_yaw thumb_pitch index middle ring pinky.',
            )
            return None
        try:
            return tuple(float(value) for value in raw)
        except ValueError:
            QMessageBox.warning(self._widget, 'G1 Inspire Hand Control', 'Manual angles must be numbers.')
            return None

    def send_manual_action(self) -> None:
        if self._busy or self._manual_slider_worker_running or not self.apply_duration():
            return
        self._capture_output_options()
        values = self.parse_manual_values()
        if values is None:
            return
        self._set_manual_slider_values(values)
        hand = self.current_hand()
        self.run_action(
            'Manual',
            lambda: self._send_manual_values(hand, values),
            apply_duration=False,
        )

    def _send_manual_values(self, hand: str, values) -> None:
        if self._send_values(hand, values, 'manual'):
            self._current_phase = 'manual'
            print(f'manual: {hand}')
            print(format_command_values(values))

    def _slider_values(self) -> tuple[float, ...]:
        return tuple(slider.value() / SLIDER_SCALE for slider in self._manual_sliders)

    def _set_manual_slider_values(self, values: Sequence[float]) -> None:
        self._manual_slider_updating = True
        try:
            for slider, label, value, max_value in zip(
                self._manual_sliders,
                self._manual_value_labels,
                values,
                MANUAL_SLIDER_MAX_VALUES,
            ):
                slider_value = min(max(float(value), MANUAL_SLIDER_MIN), max_value)
                slider.setValue(int(round(slider_value * SLIDER_SCALE)))
                label.setText(f'{slider_value:.1f}')
            self._manual.setText(' '.join(f'{float(value):.3f}' for value in values))
        finally:
            self._manual_slider_updating = False

    def _on_manual_slider_change(self, index: int) -> None:
        values = self._slider_values()
        self._manual_value_labels[index].setText(f'{values[index]:.1f}')
        self._manual.setText(' '.join(f'{value:.3f}' for value in values))
        if self._manual_slider_updating:
            return
        self._queue_manual_slider_send()

    def _adjust_manual_slider(self, index: int, delta_ticks: int) -> None:
        if self._busy or self._manual_slider_worker_running:
            return
        slider = self._manual_sliders[index]
        slider.setValue(min(max(slider.value() + delta_ticks, slider.minimum()), slider.maximum()))
        slider.setFocus()

    def _focus_manual_slider(self, index: int) -> None:
        index = min(max(index, 0), len(self._manual_sliders) - 1)
        self._manual_sliders[index].setFocus()

    def _queue_manual_slider_send(self) -> None:
        if self._busy:
            return
        self._manual_slider_pending = self._slider_values()
        if self._manual_slider_worker_running:
            return
        QTimer.singleShot(80, self._start_pending_manual_slider_send)

    def _start_pending_manual_slider_send(self) -> None:
        if self._busy or self._manual_slider_worker_running or self._manual_slider_pending is None:
            return
        values = self._manual_slider_pending
        self._manual_slider_pending = None
        hand = self.current_hand()
        self._manual_slider_worker_running = True
        self._status_label.setText('Running Manual slider...')

        duration_sec = min(float(self._duration.value()), MANUAL_SLIDER_DURATION_SEC)
        send_action = self._send_action.isChecked()
        send_topic = self._send_topic.isChecked()

        def worker() -> None:
            error = None
            success = False
            try:
                success = self._send_values(
                    hand,
                    values,
                    'manual-slider',
                    duration_sec=duration_sec,
                    send_action=send_action,
                    send_topic=send_topic,
                )
                if success:
                    self._current_phase = 'manual'
                    print(f'manual-slider: {hand}')
                    print(format_command_values(values))
            except Exception as exc:  # noqa: BLE001
                error = exc
            self._signals.slider_finished.emit(error, success)

        threading.Thread(target=worker, daemon=True).start()

    def _finish_manual_slider_send(self, error, success: bool) -> None:
        self._manual_slider_worker_running = False
        if error is not None:
            self._status_label.setText('Failed: Manual slider')
            QMessageBox.critical(self._widget, 'G1 Inspire Hand Control', str(error))
            self._manual_slider_pending = None
            return
        if self._manual_slider_pending is not None:
            QTimer.singleShot(40, self._start_pending_manual_slider_send)
            return
        if success:
            self._refresh_state('Done: Manual slider')
        else:
            self._status_label.setText('Manual slider command was not accepted')

    def run_action(self, label: str, action, close_after: bool = False, apply_duration: bool = True) -> None:
        if self._busy or self._manual_slider_worker_running:
            return
        if apply_duration and not self.apply_duration():
            return
        self._capture_output_options()
        self._set_busy(True, f'Running {label}...')

        def worker() -> None:
            error = None
            try:
                action()
            except Exception as exc:  # noqa: BLE001
                error = exc
            self._signals.finished.emit(label, error, close_after)

        threading.Thread(target=worker, daemon=True).start()

    def close_both_and_quit(self) -> None:
        self.close_both_hands()

    def _make_trajectory(self, hand: str, values, duration_sec: float) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self._node.get_clock().now().to_msg()
        trajectory.joint_names = HAND_JOINT_NAMES[hand]

        point = JointTrajectoryPoint()
        point.positions = [math.radians(value) for value in values]
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        trajectory.points = [point]
        return trajectory

    def _send_values(
        self,
        hand: str,
        values,
        phase_name: str,
        duration_sec: float | None = None,
        send_action: bool | None = None,
        send_topic: bool | None = None,
    ) -> bool:
        if send_action is None:
            send_action = self._send_action_enabled
        if send_topic is None:
            send_topic = self._send_topic_enabled
        if not send_action and not send_topic:
            print('Enable Action, Topic, or both before sending commands.')
            return False

        if duration_sec is None:
            duration_sec = self._duration_sec
        trajectory = self._make_trajectory(hand, values, duration_sec)

        if send_topic:
            self._trajectory_publishers[hand].publish(trajectory)
            print(f'{phase_name}: published topic /{CONTROLLER_NAMES[hand]}/joint_trajectory')

        if not send_action:
            return True

        client = self._action_clients[hand]
        action_name = f'/{CONTROLLER_NAMES[hand]}/follow_joint_trajectory'
        if not client.wait_for_server(timeout_sec=2.0):
            print(f'Action server is not available: {action_name}')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print(f'Goal rejected: {action_name}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            print(f'{phase_name}: error code {result.error_code}: {result.error_string}')
            return False
        return True

    def _finish_action(self, label: str, error, close_after: bool) -> None:
        self._set_busy(False)
        if error is not None:
            self._status_label.setText(f'Failed: {label}')
            QMessageBox.critical(self._widget, 'G1 Inspire Hand Control', str(error))
            return
        self._refresh_state(f'Done: {label}')
        if close_after:
            self._widget.close()

    def _set_busy(self, busy: bool, status: str | None = None) -> None:
        self._busy = busy
        for button in self._buttons:
            button.setEnabled(not busy)
        if status is not None:
            self._status_label.setText(status)

    def _refresh_state(self, status: str | None = None) -> None:
        if self._current_task is None:
            self._task_label.setText('No task selected')
        else:
            self._task_label.setText(f'Task: {self._current_task.name}')
        self._phase_label.setText(f'Phase: {self._current_phase}')
        self._sync_hand_buttons()
        self._sync_task_buttons()
        if status is not None:
            self._status_label.setText(status)

    def shutdown_plugin(self):
        self._widget.close()
        for client in self._action_clients.values():
            self._node.destroy_client(client)
        for publisher in self._trajectory_publishers.values():
            self._node.destroy_publisher(publisher)
        self._node.destroy_node()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('duration', self._duration.value())
        instance_settings.set_value('manual', self._manual.text())
        instance_settings.set_value('send_action', self._send_action.isChecked())
        instance_settings.set_value('send_topic', self._send_topic.isChecked())

    def restore_settings(self, plugin_settings, instance_settings):
        duration = instance_settings.value('duration')
        if duration is not None:
            try:
                self._duration.setValue(float(duration))
            except (TypeError, ValueError):
                pass
        manual = instance_settings.value('manual')
        if manual is not None:
            self._manual.setText(str(manual))
        send_action = instance_settings.value('send_action')
        if send_action is not None:
            self._send_action.setChecked(str(send_action).lower() in ('1', 'true'))
        send_topic = instance_settings.value('send_topic')
        if send_topic is not None:
            self._send_topic.setChecked(str(send_topic).lower() in ('1', 'true'))
