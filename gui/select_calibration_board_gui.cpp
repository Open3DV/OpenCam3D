#include "select_calibration_board_gui.h"

SelectCalibrationBoardGui::SelectCalibrationBoardGui(QWidget* parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	board_type_ = 20;
	confirm_flag_ = false;


	connect(ui.radioButton_board_20, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_board_20(bool)));
	connect(ui.radioButton_board_40, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_board_40(bool)));

	connect(ui.pushButton_ok, SIGNAL(clicked()), this, SLOT(do_pushButton_ok()));
	connect(ui.pushButton_cancel, SIGNAL(clicked()), this, SLOT(do_pushButton_cancel()));

	do_QRadioButton_toggled_board_20(true);
}

SelectCalibrationBoardGui::~SelectCalibrationBoardGui()
{
}


void SelectCalibrationBoardGui::set_board_type(int board)
{
	board_type_ = board;

	switch (board)
	{
	case 20:
	{
		ui.radioButton_board_20->setChecked(true);
		do_QRadioButton_toggled_board_20(true);
	}
	break;

	case 40:
	{
		ui.radioButton_board_40->setChecked(true);
		do_QRadioButton_toggled_board_40(true);
	}
	break;
	default:
		break;
	}
}

int SelectCalibrationBoardGui::get_board_type()
{
	return board_type_;
}


bool SelectCalibrationBoardGui::is_confirm()
{
	return confirm_flag_;
}

void SelectCalibrationBoardGui::do_QRadioButton_toggled_board_20(bool state)
{
	board_type_ = 20;

	QString str = "";
	str += u8"  非对称圆标定板";
	str += "\r\n";
	str += "\r\n";
	str += u8"  圆点数: 7*11";
	str += "\r\n";
	str += "\r\n";
	str += u8"  圆心距: 20 mm";

	ui.label_board_message->setText(str);
}

void SelectCalibrationBoardGui::do_QRadioButton_toggled_board_40(bool state)
{
	board_type_ = 40;

	QString str = "";
	str += u8"  非对称圆标定板";
	str += "\r\n";
	str += "\r\n";
	str += u8"  圆点数: 7*11";
	str += "\r\n";
	str += "\r\n";
	str += u8"  圆心距: 40 mm";

	ui.label_board_message->setText(str);
}

void SelectCalibrationBoardGui::do_pushButton_ok()
{
	confirm_flag_ = true;

	this->accept();
}

void SelectCalibrationBoardGui::do_pushButton_cancel()
{
	confirm_flag_ = false;
	this->reject();
}