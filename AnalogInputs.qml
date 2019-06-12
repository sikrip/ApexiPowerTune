import QtQuick 2.8
import QtQuick.Extras 1.4
import QtQuick.Controls 2.1
import QtQuick.Controls.Styles 1.4

Rectangle {
    anchors.fill: parent
    color: "grey"
    id: main
    Grid {

        rows:12
        columns: 3
        spacing: 5
        Text { text: "  "
            font.pixelSize: main.width / 55;color:"white"}
        Text { text: "Value @ 0V"
            font.pixelSize: main.width / 55;color:"white"}
        Text { text: "Value @ 5V"
            font.pixelSize: main.width / 55;color:"white"}
        Text { text: "Analog 0"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 1"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 2"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 3"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 4"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 5"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 6"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 7"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 8"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 9"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        Text { text: "Analog 10"
            font.pixelSize: main.width / 55;color:"white"}
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "0"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
        TextField {
            //id: maxRPM
            width: main.width / 5.5
            height: main.height /15
            font.pixelSize: main.width / 55
            placeholderText: "5"
            inputMethodHints: Qt.ImhFormattedNumbersOnly  // this ensures valid inputs are number only
            //onEditingFinished: Dashboard.setmaxRPM(maxRPM.text)
            //Component.onCompleted: Dashboard.setmaxRPM(maxRPM.text)
        }
    }
}
