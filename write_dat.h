#include "gps_lib.h"

typedef enum{
	Z = 0,
	SUB_ID,
	WN,
	L2_CODE,
	URA,
	SV_HEALTH,
	M_IODC,
	L2_FLAG,
	TGD,
	L_IODC,
	TOC,
	A2,
	A1,
	A0, //subframe 1 ends
	ZZ,
	SUB_ID2,
	IODE,
	C_RS,
	DELN,
	M_M0,
	L_M0,
	C_UC,
	MSB_E,
	LSB_E,
	C_US,
	M_SQRTA,
	L_SQRTA,
	TOE,
	AODO, //subframe 2 ends
	ZZZ,
	SUB_ID3,
	C_IC,
	M_OMG0,
	L_OMG0,
	C_IS,
	M_I0,
	L_I0,
	C_RC,
	M_OMG,
	L_OMG,
	OMGD,
	IODE2,
	IDOT, //subframe 3 ends
	ZZZZ,
	SUB_ID4,
	ZZZZZ,
	SUB_ID5,
	LAST_1
}subframe1_ele; 

typedef struct {
	int ori_eph; //the true data in the navagation message
	int pos;     //data start place in the navagation message
	int width;   //data width in the navagation message
}_ori_data; //original data in nav

int find_eph_by_toe(int toe, int *id, nav_t *nav);
int init_dat_files(int length, char *dat);
int init_original_data(nav_t *nav, /*eph_t *eph, */_ori_data *data_stream, int n);
char *itoa(int val, char *buf, unsigned radix, int length);
char *itoa_with_len(int val, char *buf, unsigned radix, int length);
int convert2string(char eph[][33], _ori_data *data);
int update_dat_structure(char *dat, char eph[][33], _ori_data *dat_info);
int make_frame(nav_t *nav, int n, int min, char *filename);
int check_parity_all(char *nav_dat);
int check_parity(char *nav_dat, int zero_flag);
