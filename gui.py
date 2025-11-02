#!/usr/bin/env python3
# main.py
import sys
import os
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFileDialog, QComboBox, QDoubleSpinBox, QCheckBox, QScrollArea,
    QWidgetItem, QMessageBox, QFrame
)
from PyQt5.QtCore import Qt

class ReplayGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Replay Mode JSON Configurator")
        self.resize(700, 520)

        self.input_folder = ""
        self.output_folder = ""
        self.service_checkboxes = []  # list of (dict_item_ref, QCheckBox)

        layout = QVBoxLayout()
        self.setLayout(layout)

        # Mode selection
        mode_h = QHBoxLayout()
        mode_h.addWidget(QLabel("Mode:"))
        self.mode_box = QComboBox()
        self.mode_box.addItems(["로그 리플레이 모드", "DB 리플레이 모드"])
        self.mode_box.currentIndexChanged.connect(self.on_mode_changed)
        mode_h.addWidget(self.mode_box)
        layout.addLayout(mode_h)

        # Input / Output folder selectors
        f_h = QHBoxLayout()
        self.input_label = QLabel("Input JSON 폴더: (선택 안됨)")
        btn_in = QPushButton("입력 폴더 선택")
        btn_in.clicked.connect(self.choose_input_folder)
        f_h.addWidget(self.input_label, 1)
        f_h.addWidget(btn_in)
        layout.addLayout(f_h)

        f2_h = QHBoxLayout()
        self.output_label = QLabel("Output 폴더: (선택 안됨)")
        btn_out = QPushButton("출력 폴더 선택")
        btn_out.clicked.connect(self.choose_output_folder)
        f2_h.addWidget(self.output_label, 1)
        f2_h.addWidget(btn_out)
        layout.addLayout(f2_h)

        # Time Scale Ratio (for 로그 모드)
        t_h = QHBoxLayout()
        self.time_ratio_label = QLabel("Time Scale Ratio:")
        self.time_ratio_spin = QDoubleSpinBox()
        self.time_ratio_spin.setRange(0.1, 1.0)
        self.time_ratio_spin.setSingleStep(0.1)
        self.time_ratio_spin.setValue(1.0)
        t_h.addWidget(self.time_ratio_label)
        t_h.addWidget(self.time_ratio_spin)
        t_h.addStretch()
        layout.addLayout(t_h)

        # SERVICE items area (scrollable)
        layout.addWidget(QLabel("SERVICE Items (로그 모드에서 사용)"))
        self.service_area = QScrollArea()
        self.service_area.setWidgetResizable(True)
        self.service_inner = QWidget()
        self.service_inner_layout = QVBoxLayout()
        self.service_inner.setLayout(self.service_inner_layout)
        self.service_area.setWidget(self.service_inner)
        self.service_area.setMinimumHeight(180)
        layout.addWidget(self.service_area)

        # Buttons: Load JSONs, Apply & Save
        btn_h = QHBoxLayout()
        btn_load = QPushButton("입력 폴더의 JSON 읽어오기")
        btn_load.clicked.connect(self.load_jsons_from_input)
        btn_apply = QPushButton("적용 및 저장 (출력 폴더로)")
        btn_apply.clicked.connect(self.apply_and_save)
        btn_h.addWidget(btn_load)
        btn_h.addWidget(btn_apply)
        layout.addLayout(btn_h)

        # Status / notes
        self.status_label = QLabel("")
        layout.addWidget(self.status_label)

        # initial UI state
        self.on_mode_changed(0)

    def choose_input_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "입력 JSON 폴더 선택", os.getcwd())
        if folder:
            self.input_folder = folder
            self.input_label.setText(f"Input JSON 폴더: {folder}")

    def choose_output_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "출력 폴더 선택", os.getcwd())
        if folder:
            self.output_folder = folder
            self.output_label.setText(f"Output 폴더: {folder}")

    def on_mode_changed(self, idx):
        mode = self.mode_box.currentText()
        is_log = "로그" in mode
        self.time_ratio_label.setVisible(is_log)
        self.time_ratio_spin.setVisible(is_log)
        self.service_area.setVisible(is_log)

    def clear_service_checkboxes(self):
        # clear layout
        for i in reversed(range(self.service_inner_layout.count())):
            item = self.service_inner_layout.takeAt(i)
            w = item.widget()
            if w:
                w.deleteLater()
        self.service_checkboxes = []

    def load_jsons_from_input(self):
        if not self.input_folder:
            QMessageBox.warning(self, "경고", "먼저 입력 폴더를 선택하세요.")
            return

        mode = self.mode_box.currentText()
        try:
            if "로그" in mode:
                # Load System.json (for Time Scale and SERVICE items)
                sys_path = os.path.join(self.input_folder, "System.json")
                if not os.path.isfile(sys_path):
                    QMessageBox.warning(self, "파일 없음", f"System.json이 입력 폴더에 없습니다:\n{sys_path}")
                    return
                with open(sys_path, "r", encoding="utf-8") as f:
                    self.sys_data = json.load(f)
                # Time scale
                ts = None
                if isinstance(self.sys_data, dict):
                    # 안전하게 경로 접근
                    ts = (self.sys_data.get("Replay Mode") or {}).get("Time Scale Ratio")
                if ts is not None:
                    try:
                        self.time_ratio_spin.setValue(float(ts))
                    except Exception:
                        pass

                # SERVICE items -> build checkboxes
                self.clear_service_checkboxes()
                svc_items = None
                if isinstance(self.sys_data, dict):
                    svc_items = (self.sys_data.get("SERVICE") or {}).get("Items")
                if isinstance(svc_items, list):
                    for item in svc_items:
                        name = item.get("Name", "<no name>")
                        enabled = bool(item.get("Enable", False))
                        chk = QCheckBox(name)
                        chk.setChecked(enabled)
                        # store reference to the underlying dict so we can update when saving
                        self.service_inner_layout.addWidget(chk)
                        self.service_checkboxes.append((item, chk))
                else:
                    self.service_inner_layout.addWidget(QLabel("SERVICE.Items가 없거나 형식이 올바르지 않습니다."))

                # Also load Service.json (so we can preview UA LOG_LOAD if needed)
                svc_path = os.path.join(self.input_folder, "Service.json")
                if os.path.isfile(svc_path):
                    with open(svc_path, "r", encoding="utf-8") as f:
                        self.service_json = json.load(f)
                else:
                    self.service_json = None

                self.status_label.setText("System.json / Service.json 로드 완료.")
            else:
                # DB 모드: load Sensor.json
                sensor_path = os.path.join(self.input_folder, "Sensor.json")
                if not os.path.isfile(sensor_path):
                    QMessageBox.warning(self, "파일 없음", f"Sensor.json이 입력 폴더에 없습니다:\n{sensor_path}")
                    return
                with open(sensor_path, "r", encoding="utf-8") as f:
                    self.sensor_json = json.load(f)
                self.status_label.setText("Sensor.json 로드 완료.")
                self.clear_service_checkboxes()
        except Exception as e:
            QMessageBox.critical(self, "오류", f"JSON 로드 중 오류가 발생했습니다:\n{e}")

    def apply_and_save(self):
        if not self.input_folder or not self.output_folder:
            QMessageBox.warning(self, "경고", "입력 폴더와 출력 폴더를 모두 선택하세요.")
            return

        mode = self.mode_box.currentText()
        try:
            if "로그" in mode:
                self._apply_log_mode()
            else:
                self._apply_db_mode()
        except Exception as e:
            QMessageBox.critical(self, "오류", f"적용/저장 중 오류가 발생했습니다:\n{e}")
            return

        QMessageBox.information(self, "완료", "수정된 JSON들이 출력 폴더에 저장되었습니다.")
        self.status_label.setText("저장 완료.")

    def _apply_log_mode(self):
        # System.json
        sys_path = os.path.join(self.input_folder, "System.json")
        with open(sys_path, "r", encoding="utf-8") as f:
            sys_data = json.load(f)

        # set Time Scale Ratio
        ts = float(self.time_ratio_spin.value())
        if "Replay Mode" in sys_data and isinstance(sys_data["Replay Mode"], dict):
            sys_data["Replay Mode"]["Time Scale Ratio"] = ts
        else:
            # 안전하게 넣기
            sys_data["Replay Mode"] = {"Time Scale Ratio": ts}

        # SERVICE items - checkboxes map back to items
        for item_ref, chk in self.service_checkboxes:
            item_ref["Enable"] = bool(chk.isChecked())

        # write System.json to output folder
        out_sys = os.path.join(self.output_folder, "System.json")
        with open(out_sys, "w", encoding="utf-8") as f:
            json.dump(sys_data, f, ensure_ascii=False, indent=4)

        # Service.json: update UA LOG_LOAD parameter value (set to input folder path)
        svc_path = os.path.join(self.input_folder, "Service.json")
        if os.path.isfile(svc_path):
            with open(svc_path, "r", encoding="utf-8") as f:
                svc_data = json.load(f)
            for item in svc_data.get("Items", []):
                if item.get("Name") == "UA LOG_LOAD":
                    # safe navigation for Parameter -> Value
                    if "Parameter" in item and isinstance(item["Parameter"], dict):
                        # many JSONs may have Parameter as list or dict; handle common patterns
                        if "Value" in item["Parameter"]:
                            item["Parameter"]["Value"] = self.input_folder
                        else:
                            # try if Parameter is list of dicts
                            if isinstance(item["Parameter"], list):
                                for p in item["Parameter"]:
                                    if isinstance(p, dict) and "Value" in p:
                                        p["Value"] = self.input_folder
                    # also check if Parameter is list outside
                    if isinstance(item.get("Parameters"), list):
                        for p in item["Parameters"]:
                            if isinstance(p, dict) and "Value" in p:
                                p["Value"] = self.input_folder
            # write to output
            out_svc = os.path.join(self.output_folder, "Service.json")
            with open(out_svc, "w", encoding="utf-8") as f:
                json.dump(svc_data, f, ensure_ascii=False, indent=4)
        else:
            # If Service.json missing, still write System.json and warn
            QMessageBox.information(self, "알림", "입력 폴더에 Service.json이 없습니다. System.json만 저장됩니다.")

    def _apply_db_mode(self):
        sensor_path = os.path.join(self.input_folder, "Sensor.json")
        with open(sensor_path, "r", encoding="utf-8") as f:
            sensor_data = json.load(f)

        # Example: enable only certain Types
        enable_types = {"LIDAR", "RADAR", "CAMERA"}  # 필요시 수정 가능

        for item in sensor_data.get("Items", []):
            t = item.get("Type", "")
            if t in enable_types:
                item["Enabled"] = True
            else:
                item["Enabled"] = False

            # Set Logging File (or LoggingFile) to input folder path if field exists or create
            if "Logging File" in item:
                item["Logging File"] = self.input_folder
            elif "LoggingFile" in item:
                item["LoggingFile"] = self.input_folder
            else:
                # create a common key
                item["Logging File"] = self.input_folder

        out_sensor = os.path.join(self.output_folder, "Sensor.json")
        with open(out_sensor, "w", encoding="utf-8") as f:
            json.dump(sensor_data, f, ensure_ascii=False, indent=4)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ReplayGUI()
    w.show()
    sys.exit(app.exec_())