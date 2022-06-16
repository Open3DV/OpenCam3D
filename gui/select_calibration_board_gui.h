#pragma once

#include <QDialog>
#include "ui_select_calibration_board_gui.h"

class SelectCalibrationBoardGui : public QDialog
{
	Q_OBJECT

public:
	SelectCalibrationBoardGui(QWidget *parent = Q_NULLPTR);
	~SelectCalibrationBoardGui();

	int get_board_type();

	void set_board_type(int board);

	bool is_confirm();

private slots:
	void do_QRadioButton_toggled_board_20(bool state);

	void do_QRadioButton_toggled_board_40(bool state);

	void do_pushButton_ok();

	void do_pushButton_cancel();


private:
	Ui::SelectCalibrationBoardGui ui;

	int board_type_;

	bool confirm_flag_;
};
