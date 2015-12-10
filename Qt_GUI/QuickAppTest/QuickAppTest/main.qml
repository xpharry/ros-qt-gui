import QtQuick 2.3
import QtQuick.Window 2.2

Window {
    visible: true

    width: 640
    height: 480

    MouseArea {
        anchors.fill: parent
        onClicked: {
            Qt.quit();
        }

        Item {
            x: 227
            y: 262
            width: 200
            height: 200
        }
    }

    Text {
        text: qsTr("Hello World")
        anchors.centerIn: parent
    }
}

