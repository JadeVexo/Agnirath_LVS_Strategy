import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget

app = QApplication(sys.argv)

main_window = QMainWindow()
main_window.setWindowTitle('PyQt Table Example')
main_window.setGeometry(100, 100, 600, 400)

central_widget = QWidget(main_window)
main_window.setCentralWidget(central_widget)

layout = QVBoxLayout(central_widget)

table_widget = QTableWidget()
layout.addWidget(table_widget)

table_widget.setRowCount(5)
table_widget.setColumnCount(3)

# Hide row and column headers
table_widget.setVerticalHeaderLabels(['hi'] * 5)  # Empty labels for rows
table_widget.setHorizontalHeaderLabels(['bye'] * 3)  # Empty labels for columns
# table_widget.setStyleSheet("background-color: transparent;")
table_widget.setStyleSheet("QTableWidget::item { border: 1px solid black; }")
# Populate the table with custom text
data = [
    ['Item 1', 'Item 2', 'Item 3'],
    ['Item 4', 'Item 5', 'Item 6'],
    ['Item 7', 'Item 8', 'Item 9'],
    ['Item 10', 'Item 11', 'Item 12'],
    ['Item 13', 'Item 14', 'Item 15']
]

for row in range(5):
    for col in range(3):
        item = QTableWidgetItem(data[row][col])
        table_widget.setItem(row, col, item)

main_window.show()

sys.exit(app.exec_())
# import sys
# from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget
# from PyQt5.QtGui import QColor

# app = QApplication(sys.argv)

# main_window = QMainWindow()
# main_window.setWindowTitle('PyQt Table Example')
# main_window.setGeometry(100, 100, 600, 400)

# central_widget = QWidget(main_window)
# main_window.setCentralWidget(central_widget)

# layout = QVBoxLayout(central_widget)

# table_widget = QTableWidget()
# layout.addWidget(table_widget)

# table_widget.setRowCount(5)
# table_widget.setColumnCount(3)

# # Hide row and column headers
# table_widget.verticalHeader().setVisible(False)
# table_widget.horizontalHeader().setVisible(False)

# # Set cell background color
# for row in range(5):
#     for col in range(3):
#         item = QTableWidgetItem(f'Item {row + 1}-{col + 1}')
#         item.setBackground(QColor(255, 0, 0))  # Red background color
#         table_widget.setItem(row, col, item)

# # Set grid line color
# table_widget.setStyleSheet("QTableWidget::item { border: 1px solid black; }")

# # Set the parent widget's background to transparent
# central_widget.setStyleSheet("background-color: transparent;")

# main_window.show()

# sys.exit(app.exec_())
