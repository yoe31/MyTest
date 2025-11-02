import sys
import json
import os
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFileDialog, QComboBox, QDoubleSpinBox, QCheckBox, QListWidget, QMessageBox
)

class ReplayGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Replay Mode JSON Configurator")
        self.resize(600, 400)

        layout = QVBoxLayout()
        self.setLayout(layout)

        # 모드 선택
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self.mode_box = QComboBox()
        self.mode_box.addItems(["로그 리플레이 모드", "DB 리플레이 모드"])
        self.mode_box.currentIndexChanged.connect(self.mode_changed)
        mode_layout.addWidget(self.mode_box)
        layout.addLayout(mode_layout)

        # Time Scale Ratio
        self.time_ratio_label = QLabel("Time Scale Ratio:")
        self.time_ratio_spin = QDoubleSpinBox()
        self.time_ratio_spin.setRange(0.1, 1.0)
        self.time_ratio_spin.setSingleStep(0.1)
        self.time_ratio_spin.setValue(1.0)
        layout.addWidget(self.time_ratio_label)
        layout.addWidget(self.time_ratio_spin)

        # SERVICE 항목 리스트
        self.service_label = QLabel("SERVICE Items (Enable 선택):")
        self.service_list = QListWidget()
        layout.addWidget(self.service_label)
        layout.addWidget(self.service_list)

        # 폴더 선택 버튼
        self.folder_btn = QPushButton("폴더 선택")
        self.folder_btn.clicked.connect(self.select_folder)
        layout.addWidget(self.folder_btn)

        # 실행 버튼
        self.apply_btn = QPushButton("적용 및 저장")
        self.apply_btn.clicked.connect(self.apply_changes)
        layout.addWidget(self.apply_btn)

        # 내부 변수
        self.selected_folder = ""
        self.mode_changed(0)

    def mode_changed(self, idx):
        """모드 전환 시 UI 표시 조정"""
        mode = self.mode_box.currentText()
        if "로그" in mode:
            self.time_ratio_label.show()
            self.time_ratio_spin.show()
            self.service_label.show()
            self.service_list.show()
        else:
            self.time_ratio_label.hide()
            self.time_ratio_spin.hide()
            self.service_label.hide()
            self.service_list.hide()

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "폴더 선택")
        if folder:
            self.selected_folder = folder
            QMessageBox.information(self, "선택됨", f"폴더: {folder}")

    def apply_changes(self):
        mode = self.mode_box.currentText()

        if not self.selected_folder:
            QMessageBox.warning(self, "경고", "폴더를 먼저 선택하세요.")
            return

        output_dir = os.path.join(os.getcwd(), "output")
        os.makedirs(output_dir, exist_ok=True)

        if "로그" in mode:
            self.modify_log_mode(output_dir)
        else:
            self.modify_db_mode(output_dir)

        QMessageBox.information(self, "완료", "수정된 파일이 저장되었습니다.")

    def modify_log_mode(self, output_dir):
        # 1. System.json 수정
        sys_path = "json/System.json"
        with open(sys_path, "r", encoding="utf-8") as f:
            sys_data = json.load(f)

        # Time Scale Ratio 설정
        if "Replay Mode" in sys_data:
            sys_data["Replay Mode"]["Time Scale Ratio"] = self.time_ratio_spin.value()

        # SERVICE Items Enable 조정
        if "SERVICE" in sys_data and "Items" in sys_data["SERVICE"]:
            self.service_list.clear()
            for item in sys_data["SERVICE"]["Items"]:
                name = item.get("Name", "")
                chk = QCheckBox(name)
                chk.setChecked(item.get("Enable", False))
                self.service_list.addItem(name)
                item["Enable"] = chk.isChecked()

        out_path = os.path.join(output_dir, "System.json")
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(sys_data, f, ensure_ascii=False, indent=4)

        # 2. Service.json 수정
        svc_path = "json/Service.json"
        with open(svc_path, "r", encoding="utf-8") as f:
            svc_data = json.load(f)

        for item in svc_data.get("Items", []):
            if item.get("Name") == "UA LOG_LOAD":
                if "Parameter" in item and "Value" in item["Parameter"]:
                    item["Parameter"]["Value"] = self.selected_folder

        out_path = os.path.join(output_dir, "Service.json")
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(svc_data, f, ensure_ascii=False, indent=4)

    def modify_db_mode(self, output_dir):
        sensor_path = "json/Sensor.json"
        with open(sensor_path, "r", encoding="utf-8") as f:
            sensor_data = json.load(f)

        for item in sensor_data.get("Items", []):
            if item.get("Type") in ["LIDAR", "RADAR", "CAMERA"]:  # 예시로 특정 Type만 true
                item["Enabled"] = True
            else:
                item["Enabled"] = False

            if "Logging File" in item:
                item["Logging File"] = self.selected_folder

        out_path = os.path.join(output_dir, "Sensor.json")
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(sensor_data, f, ensure_ascii=False, indent=4)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ReplayGUI()
    window.show()
    sys.exit(app.exec_())