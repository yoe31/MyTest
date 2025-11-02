#!/usr/bin/env python3
# main.py
import sys
import os
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFileDialog, QComboBox, QDoubleSpinBox, QCheckBox, QScrollArea,
    QMessageBox, QWidgetItem
)
from PyQt5.QtCore import Qt

class ReplayGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Replay Mode JSON Configurator")
        self.resize(780, 560)

        # folders
        self.input_folder = ""
        self.output_folder = ""
        self.log_folder = ""  # 추가: UA_LOG_LOAD_Service에 넣을 로그 경로

        # data holders
        self.sys_data = None
        self.service_json = None
        self.sensor_json = None
        self.service_checkboxes = []  # list of (item_ref, QCheckBox)

        layout = QVBoxLayout()
        self.setLayout(layout)

        # Mode selection
        mode_h = QHBoxLayout()
        mode_h.addWidget(QLabel("Mode:"))
        self.mode_box = QComboBox()
        self.mode_box.addItems(["로그 리플레이 모드", "DB 리플레이 모드"])
        self.mode_box.currentIndexChanged.connect(self.on_mode_changed)
        mode_h.addWidget(self.mode_box)
        mode_h.addStretch()
        layout.addLayout(mode_h)

        # Input folder selector
        f_in_h = QHBoxLayout()
        self.input_label = QLabel("Input JSON 폴더: (선택 안됨)")
        btn_in = QPushButton("입력 폴더 선택")
        btn_in.clicked.connect(self.choose_input_folder)
        f_in_h.addWidget(self.input_label, 1)
        f_in_h.addWidget(btn_in)
        layout.addLayout(f_in_h)

        # Output folder selector
        f_out_h = QHBoxLayout()
        self.output_label = QLabel("Output 폴더: (선택 안됨)")
        btn_out = QPushButton("출력 폴더 선택")
        btn_out.clicked.connect(self.choose_output_folder)
        f_out_h.addWidget(self.output_label, 1)
        f_out_h.addWidget(btn_out)
        layout.addLayout(f_out_h)

        # Log folder selector (for UA_LOG_LOAD_Service)
        f_log_h = QHBoxLayout()
        self.log_label = QLabel("로그 폴더 (UA_LOG_LOAD_Service용): (선택 안됨)")
        btn_log = QPushButton("로그 폴더 선택")
        btn_log.clicked.connect(self.choose_log_folder)
        f_log_h.addWidget(self.log_label, 1)
        f_log_h.addWidget(btn_log)
        layout.addLayout(f_log_h)

        # Time Scale Ratio (로그 모드)
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
        layout.addWidget(QLabel("SERVICE Items (로그 모드에서 체크하여 Enable 조정)"))
        self.service_area = QScrollArea()
        self.service_area.setWidgetResizable(True)
        self.service_inner = QWidget()
        self.service_inner_layout = QVBoxLayout()
        self.service_inner.setLayout(self.service_inner_layout)
        self.service_area.setWidget(self.service_inner)
        self.service_area.setMinimumHeight(180)
        layout.addWidget(self.service_area)

        # VIEW_MODE LMG_A 설정 (모든 모드에서 적용)
        view_h = QHBoxLayout()
        view_h.addWidget(QLabel("Service.json -> VIEW_MODE 안의 LMG_A 값 설정:"))
        self.lmg_combo = QComboBox()
        self.lmg_combo.addItems(["0", "1", "2"])
        view_h.addWidget(self.lmg_combo)
        view_h.addStretch()
        layout.addLayout(view_h)

        # Buttons: Load and Apply
        btn_h = QHBoxLayout()
        btn_load = QPushButton("입력 폴더의 JSON 읽어오기")
        btn_load.clicked.connect(self.load_jsons_from_input)
        btn_apply = QPushButton("적용 및 저장 (출력 폴더로)")
        btn_apply.clicked.connect(self.apply_and_save)
        btn_h.addWidget(btn_load)
        btn_h.addWidget(btn_apply)
        layout.addLayout(btn_h)

        # Status
        self.status_label = QLabel("")
        layout.addWidget(self.status_label)

        # initial UI state
        self.on_mode_changed(0)

    # --- folder selection callbacks ---
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

    def choose_log_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "로그 폴더 선택 (UA_LOG_LOAD_Service)", os.getcwd())
        if folder:
            self.log_folder = folder
            self.log_label.setText(f"로그 폴더: {folder}")

    # --- UI mode change ---
    def on_mode_changed(self, idx):
        mode = self.mode_box.currentText()
        is_log = "로그" in mode
        self.time_ratio_label.setVisible(is_log)
        self.time_ratio_spin.setVisible(is_log)
        self.service_area.setVisible(is_log)
        # log_folder selection should be visible only in 로그 모드 (요구사항에 따라)
        self.log_label.setVisible(is_log)
        # find the log folder button visibility via parent layout: hide/show its sibling
        # (we hid label; button sits next to it so label hide is sufficient visually)

    # --- service checkbox helpers ---
    def clear_service_checkboxes(self):
        for i in reversed(range(self.service_inner_layout.count())):
            item = self.service_inner_layout.takeAt(i)
            w = item.widget()
            if w:
                w.deleteLater()
        self.service_checkboxes = []

    # --- load JSONs from input folder ---
    def load_jsons_from_input(self):
        if not self.input_folder:
            QMessageBox.warning(self, "경고", "먼저 입력 폴더를 선택하세요.")
            return

        mode = self.mode_box.currentText()
        try:
            if "로그" in mode:
                # System.json
                sys_path = os.path.join(self.input_folder, "System.json")
                if not os.path.isfile(sys_path):
                    QMessageBox.warning(self, "파일 없음", f"System.json이 입력 폴더에 없습니다:\n{sys_path}")
                    return
                with open(sys_path, "r", encoding="utf-8") as f:
                    self.sys_data = json.load(f)
                # Time Scale 설정 반영 (가능하면)
                try:
                    ts = (self.sys_data.get("Replay Mode") or {}).get("Time Scale Ratio")
                    if ts is not None:
                        self.time_ratio_spin.setValue(float(ts))
                except Exception:
                    pass

                # SERVICE.Items -> 체크박스 생성
                self.clear_service_checkboxes()
                svc_items = (self.sys_data.get("SERVICE") or {}).get("Items")
                if isinstance(svc_items, list):
                    for item in svc_items:
                        name = item.get("Name", "<no name>")
                        enabled = bool(item.get("Enable", False))
                        chk = QCheckBox(name)
                        chk.setChecked(enabled)
                        self.service_inner_layout.addWidget(chk)
                        self.service_checkboxes.append((item, chk))
                else:
                    self.service_inner_layout.addWidget(QLabel("SERVICE.Items가 없습니다."))

                # Service.json 로드 (선택적으로)
                svc_path = os.path.join(self.input_folder, "Service.json")
                if os.path.isfile(svc_path):
                    with open(svc_path, "r", encoding="utf-8") as f:
                        self.service_json = json.load(f)
                    # VIEW_MODE -> LMG_A 값 있으면 콤보박스 초기값 설정
                    vm = self._find_view_mode_lmg_value(self.service_json)
                    if vm is not None:
                        try:
                            self.lmg_combo.setCurrentText(str(int(vm)))
                        except Exception:
                            pass
                else:
                    self.service_json = None

                self.status_label.setText("System.json / Service.json 로드 완료.")
            else:
                # DB 모드: Sensor.json
                sensor_path = os.path.join(self.input_folder, "Sensor.json")
                if not os.path.isfile(sensor_path):
                    QMessageBox.warning(self, "파일 없음", f"Sensor.json이 입력 폴더에 없습니다:\n{sensor_path}")
                    return
                with open(sensor_path, "r", encoding="utf-8") as f:
                    self.sensor_json = json.load(f)
                # Service.json도 로드해서 LMG_A 값 있으면 반영
                svc_path = os.path.join(self.input_folder, "Service.json")
                if os.path.isfile(svc_path):
                    with open(svc_path, "r", encoding="utf-8") as f:
                        self.service_json = json.load(f)
                    vm = self._find_view_mode_lmg_value(self.service_json)
                    if vm is not None:
                        try:
                            self.lmg_combo.setCurrentText(str(int(vm)))
                        except Exception:
                            pass
                else:
                    self.service_json = None

                self.clear_service_checkboxes()
                self.status_label.setText("Sensor.json / Service.json(선택) 로드 완료.")
        except Exception as e:
            QMessageBox.critical(self, "오류", f"JSON 로드 중 오류가 발생했습니다:\n{e}")

    # helper to find VIEW_MODE->LMG_A current value (returns None if not found)
    def _find_view_mode_lmg_value(self, service_json):
        try:
            for item in service_json.get("Items", []):
                if item.get("Name") == "VIEW_MODE":
                    # Parameter could be dict or list; handle both
                    param = item.get("Parameter") or item.get("Parameters")
                    if isinstance(param, dict):
                        # look for key name 'LMG_A' directly or nested
                        if "LMG_A" in param:
                            return param.get("LMG_A")
                        # or Parameter may be dict with list inside
                        for k, v in param.items():
                            if isinstance(v, dict) and v.get("Name") == "LMG_A" and "Value" in v:
                                return v.get("Value")
                    elif isinstance(param, list):
                        for p in param:
                            if isinstance(p, dict) and p.get("Name") == "LMG_A":
                                return p.get("Value")
        except Exception:
            pass
        return None

    # --- apply and save ---
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

    # --- log mode modifications ---
    def _apply_log_mode(self):
        # System.json 처리
        sys_path = os.path.join(self.input_folder, "System.json")
        with open(sys_path, "r", encoding="utf-8") as f:
            sys_data = json.load(f)

        # Time Scale Ratio 안전 설정
        ts = float(self.time_ratio_spin.value())
        if "Replay Mode" in sys_data and isinstance(sys_data["Replay Mode"], dict):
            sys_data["Replay Mode"]["Time Scale Ratio"] = ts
        else:
            sys_data["Replay Mode"] = {"Time Scale Ratio": ts}

        # SERVICE Items: checkbox 반영
        for item_ref, chk in self.service_checkboxes:
            item_ref["Enable"] = bool(chk.isChecked())

        # write System.json to output
        out_sys = os.path.join(self.output_folder, "System.json")
        with open(out_sys, "w", encoding="utf-8") as f:
            json.dump(sys_data, f, ensure_ascii=False, indent=4)

        # Service.json 처리: UA_LOG_LOAD_Service (또는 UA LOG_LOAD)와 VIEW_MODE->LMG_A 수정
        svc_path = os.path.join(self.input_folder, "Service.json")
        if os.path.isfile(svc_path):
            with open(svc_path, "r", encoding="utf-8") as f:
                svc_data = json.load(f)

            # 1) UA_LOG_LOAD_Service (호환: "UA LOG_LOAD" 등) Parameter.Value = self.log_folder (우선)
            # fallback: if specific log_folder not chosen, 사용할 폴더는 self.input_folder
            target_log_path = self.log_folder if self.log_folder else self.input_folder
            for item in svc_data.get("Items", []):
                name = item.get("Name", "")
                if name in ("UA_LOG_LOAD_Service", "UA LOG_LOAD", "UA_LOG_LOAD", "UA-LOG-LOAD"):
                    # Parameter could be dict or list; handle common patterns
                    if "Parameter" in item and isinstance(item["Parameter"], dict):
                        if "Value" in item["Parameter"]:
                            item["Parameter"]["Value"] = target_log_path
                        else:
                            # set a sensible field
                            item["Parameter"]["Value"] = target_log_path
                    elif "Parameter" in item and isinstance(item["Parameter"], list):
                        for p in item["Parameter"]:
                            if isinstance(p, dict) and "Value" in p:
                                p["Value"] = target_log_path
                    # also check "Parameters"
                    if "Parameters" in item and isinstance(item["Parameters"], list):
                        for p in item["Parameters"]:
                            if isinstance(p, dict) and "Value" in p:
                                p["Value"] = target_log_path

            # 2) VIEW_MODE -> Parameter 안에서 Name == "LMG_A" 인 항목 찾아 Value를 LMG 선택값으로 변경
            self._set_view_mode_lmg_value(svc_data, int(self.lmg_combo.currentText()))

            # write modified Service.json to output
            out_svc = os.path.join(self.output_folder, "Service.json")
            with open(out_svc, "w", encoding="utf-8") as f:
                json.dump(svc_data, f, ensure_ascii=False, indent=4)
        else:
            QMessageBox.information(self, "알림", "입력 폴더에 Service.json이 없습니다. System.json만 저장됩니다.")

    # --- db mode modifications ---
    def _apply_db_mode(self):
        sensor_path = os.path.join(self.input_folder, "Sensor.json")
        with open(sensor_path, "r", encoding="utf-8") as f:
            sensor_data = json.load(f)

        # 예시: 활성화할 타입들 (GUI로 편집 가능하게 확장할 수도 있음)
        enable_types = {"LIDAR", "RADAR", "CAMERA"}

        for item in sensor_data.get("Items", []):
            t = item.get("Type", "")
            item["Enabled"] = True if t in enable_types else False
            # Logging File 또는 LoggingFile 키 처리
            if "Logging File" in item:
                item["Logging File"] = self.input_folder
            elif "LoggingFile" in item:
                item["LoggingFile"] = self.input_folder
            else:
                item["Logging File"] = self.input_folder

        # write Sensor.json
        out_sensor = os.path.join(self.output_folder, "Sensor.json")
        with open(out_sensor, "w", encoding="utf-8") as f:
            json.dump(sensor_data, f, ensure_ascii=False, indent=4)

        # Service.json도 존재하면 VIEW_MODE->LMG_A 적용 (모든 모드 공통 요구)
        svc_path = os.path.join(self.input_folder, "Service.json")
        if os.path.isfile(svc_path):
            with open(svc_path, "r", encoding="utf-8") as f:
                svc_data = json.load(f)
            self._set_view_mode_lmg_value(svc_data, int(self.lmg_combo.currentText()))
            out_svc = os.path.join(self.output_folder, "Service.json")
            with open(out_svc, "w", encoding="utf-8") as f:
                json.dump(svc_data, f, ensure_ascii=False, indent=4)

    # helper: set VIEW_MODE -> LMG_A value safely
    def _set_view_mode_lmg_value(self, service_json, new_value):
        for item in service_json.get("Items", []):
            if item.get("Name") == "VIEW_MODE":
                param = item.get("Parameter") or item.get("Parameters")
                # dict 형태
                if isinstance(param, dict):
                    # direct key
                    if "LMG_A" in param:
                        param["LMG_A"] = new_value
                        return True
                    # nested dicts
                    for k, v in param.items():
                        if isinstance(v, dict) and v.get("Name") == "LMG_A":
                            v["Value"] = new_value
                            return True
                # list 형태
                if isinstance(param, list):
                    for p in param:
                        if isinstance(p, dict) and p.get("Name") == "LMG_A":
                            p["Value"] = new_value
                            return True
                # if not found, try Parameters key
                if "Parameters" in item and isinstance(item["Parameters"], list):
                    for p in item["Parameters"]:
                        if isinstance(p, dict) and p.get("Name") == "LMG_A":
                            p["Value"] = new_value
                            return True
        return False

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ReplayGUI()
    w.show()
    sys.exit(app.exec_())