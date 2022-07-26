// Copyright (C) 2022  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 300
  Layout.minimumHeight: 350
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  // Left spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  CheckBox {
    Layout.alignment: Qt.AlignLeft
    id: showWaterPatchMarkers
    Layout.columnSpan: 4
    text: qsTr("Show water patch markers")
    checked: false
    onClicked: {
      WavesControl.OnShowWaterPatchMarkers(checked)
    }
  }

  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  CheckBox {
    Layout.alignment: Qt.AlignLeft
    id: showWaterlineMarkers
    Layout.columnSpan: 4
    text: qsTr("Show waterline markers")
    checked: false
    onClicked: {
      WavesControl.OnShowWaterlineMarkers(checked)
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignLeft
    id: showSubmergedTriangleMarkers
    Layout.columnSpan: 4
    text: qsTr("Show submerged triangle markers")
    checked: false
    onClicked: {
      WavesControl.OnShowSubmergedTriangleMarkers(checked)
    }
  }

  // wind speed
  Text {
    Layout.columnSpan: 2
    id: windSpeedText
    color: "dimgrey"
    text: "Wind speed (m/s)"
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: windSpeed
    maximumValue: 100.0
    minimumValue: 0.5
    value: 5.0
    decimals: 1
    stepSize: 0.5
    onEditingFinished: WavesControl.UpdateWindSpeed(windSpeed.value)
  }

  // wind angle
  Text {
    Layout.columnSpan: 2
    id: windAngleText
    color: "dimgrey"
    text: "Wind angle (deg)"
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: windAngle
    maximumValue: 180
    minimumValue: -180
    value: 135
    decimals: 0
    stepSize: 10
    onEditingFinished: WavesControl.UpdateWindAngle(windAngle.value)
  }

  // steepness
  Text {
    Layout.columnSpan: 2
    id: steepnessText
    color: "dimgrey"
    text: "Steepness"
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: steepness
    maximumValue: 10
    minimumValue: 0
    value: 2
    decimals: 1
    stepSize: 0.1
    onEditingFinished: WavesControl.UpdateSteepness(steepness.value)
  }

  // Bottom spacer
  Item {
    Layout.columnSpan: 4
    Layout.fillHeight: true
  }
}
