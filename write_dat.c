#include <stdio.h>
#include <stdlib.h>
#include "write_dat.h"
    
#define MAXLEN 1024
#define LEAP_SECONDS 17
#define DAT_LEN 1500
#define GPSPI 3.1415926535898

//_ori_data ori_data[LAST_1];
//char string_eph[LAST_1][33];
const int start_pos[LAST_1]={30,49,60,70,72,76,82,90,196,210,218,240,248,270, \
				330,349,360,368,390,406,420,450,466,480,510,526,540,570,587, \
				630,649,660,676,690,720,736,750,780,796,810,840,870,878, \
				930,949, \
				1230,1249};
const int bw[LAST_1]={17,3,10,2,4,6,2,1,8,8,16,8,16,22, \
			17,3,8,16,16,8,24,16,8,24,16,8,24,16,5, \
			17,3,16,8,24,16,8,24,16,8,24,24,8,14, \
			17,3, \
			17,3};

const char TLM[32] = "100010110000110011101000001010";
const	int D25[18] = {0,2,3,4,6,7,11,12,13,14,15,18,19,21,24}; //last 29 ==== 0,parity position record
const	int D26[18] = {1,3,4,5,7,8,12,13,14,15,16,19,20,22,25};
const	int D27[18] = {0,2,4,5,6,8,9,13,14,15,16,17,20,21,23};
const	int D28[18] = {1,3,5,6,7,9,10,14,15,16,17,18,21,22,24};
const	int D29[18] = {1,2,4,6,7,8,10,11,15,16,17,18,19,22,23,25};
const	int D30[18] = {0,4,6,7,9,10,11,12,14,16,20,23,24,25};

int idx[32];
char dat[2048];

int find_eph_by_toe(int toe, int *id, nav_t *nav){
	int i;
	int j = 0;
	for(i=0;i<nav->n;i++){
		if((int)nav->eph[i].toes == toe && j < 32){
			id[j] = i;
			j++;
		}
	}
	return j;
}

int init_dat_files(int length, char *dat){
	/*FILE *fp;
	fp = fopen("test.dat","wr");
	for(int i = 0;i < length; i++){
		fwrite("0", sizeof(unsigned char), 1, fp);
	}*/
	int i;
	for(i = 0;i < length; i++){
			dat[i] = '0';
	}
}

int half_adjust(double val){
	return (int)(val < 0 ? val - 0.5 : val + 0.5);	
}

int half_adjust_special(double val){
        return (unsigned int)(val < 0 ? val - 0.5 : val + 0.5);
}

int init_original_data(nav_t *nav, /*eph_t *eph, */_ori_data *data_stream, int n){
	int i;
	data_stream[Z].ori_eph = (nav->eph[n].toes + nav->leaps)/6 + 1; //TODO::leaps is uncertain
	data_stream[SUB_ID].ori_eph = 1;
	data_stream[WN].ori_eph = nav->eph[n].week - 1024;
	data_stream[L2_CODE].ori_eph = nav->eph[n].code;
	data_stream[URA].ori_eph = nav->eph[n].sva;
	data_stream[SV_HEALTH].ori_eph = nav->eph[n].svh;
	data_stream[M_IODC].ori_eph = nav->eph[n].iodc >> 8;
	data_stream[L2_FLAG].ori_eph = nav->eph[n].flag;
	data_stream[TGD].ori_eph = half_adjust(nav->eph[n].tgd[0] * pow(2,31));
	data_stream[L_IODC].ori_eph = nav->eph[n].iodc & 0x00ff;
	data_stream[TOC].ori_eph = (int)(nav->eph[n].toes / 16);
	data_stream[A2].ori_eph = half_adjust(nav->eph[n].f2 * pow(2,55)); //负数默认补码
	data_stream[A1].ori_eph = half_adjust(nav->eph[n].f1 * pow(2,43));
	data_stream[A0].ori_eph = half_adjust(nav->eph[n].f0 * pow(2,31));
	data_stream[ZZ].ori_eph = data_stream[Z].ori_eph + 1;
	data_stream[SUB_ID2].ori_eph = 2;
	data_stream[IODE2].ori_eph = nav->eph[n].iode;
	data_stream[C_RS].ori_eph = half_adjust(nav->eph[n].crs * pow(2,5));
	data_stream[DELN].ori_eph = half_adjust(nav->eph[n].deln * pow(2,43) / GPSPI);
	data_stream[M_M0].ori_eph = (half_adjust(nav->eph[n].M0 * pow(2,31) / GPSPI) >> 24) & 0xff;//TODO::decrease time
	data_stream[L_M0].ori_eph = half_adjust(nav->eph[n].M0 * pow(2,31) / GPSPI) & 0xffffff;
	data_stream[C_UC].ori_eph = half_adjust(nav->eph[n].cuc * pow(2,29));
	data_stream[MSB_E].ori_eph = (half_adjust(nav->eph[n].e * pow(2,33)) >> 24) & 0xff;
	data_stream[LSB_E].ori_eph = half_adjust(nav->eph[n].e * pow(2,33)) & 0xffffff;
	data_stream[C_US].ori_eph = half_adjust(nav->eph[n].cus * pow(2,29));
	data_stream[M_SQRTA].ori_eph = (int)((half_adjust_special(sqrt(nav->eph[n].A) * pow(2,19)) >>  24) & 0xff);
	data_stream[L_SQRTA].ori_eph = (int)(half_adjust_special(sqrt(nav->eph[n].A) * pow(2,19)) & 0xffffff);
	data_stream[TOE].ori_eph = (int)(nav->eph[n].toes / 16);
	data_stream[AODO].ori_eph = 0x1f; // default 11111
	data_stream[ZZZ].ori_eph = data_stream[ZZ].ori_eph + 1;
	data_stream[SUB_ID3].ori_eph = 3;
	data_stream[C_IC].ori_eph = half_adjust(nav->eph[n].cic * pow(2,29));
	data_stream[M_OMG0].ori_eph = (half_adjust(nav->eph[n].OMG0 * pow(2,31) / GPSPI) >> 24) & 0xff;
	data_stream[L_OMG0].ori_eph = half_adjust(nav->eph[n].OMG0 * pow(2,31) / GPSPI) & 0xffffff;
	data_stream[C_IS].ori_eph = half_adjust(nav->eph[n].cis * pow(2,29));
	data_stream[M_I0].ori_eph = (half_adjust(nav->eph[n].i0 * pow(2,31) / GPSPI) >> 24) & 0xff;
	data_stream[L_I0].ori_eph = half_adjust(nav->eph[n].i0 * pow(2,31) / GPSPI) & 0xffffff;
	data_stream[C_RC].ori_eph = half_adjust(nav->eph[n].crc * pow(2,5));
	data_stream[M_OMG].ori_eph = (half_adjust(nav->eph[n].omg * pow(2,31) / GPSPI) >> 24) & 0xff;
	data_stream[L_OMG].ori_eph = half_adjust(nav->eph[n].omg * pow(2,31) / GPSPI) & 0xffffff;
	data_stream[OMGD].ori_eph = half_adjust(nav->eph[n].OMGd * pow(2,43) / GPSPI);
	data_stream[IODE].ori_eph = nav->eph[n].iode;
	data_stream[IDOT].ori_eph = half_adjust(nav->eph[n].idot * pow(2,43) / GPSPI);
	data_stream[ZZZZ].ori_eph = data_stream[ZZZ].ori_eph + 1;
	data_stream[SUB_ID4].ori_eph = 4;
	data_stream[ZZZZZ].ori_eph = data_stream[ZZZZ].ori_eph + 1;
	data_stream[SUB_ID5].ori_eph = 5;
	for(i=0;i<LAST_1;i++){
		data_stream[i].pos = start_pos[i];
		data_stream[i].width = bw[i];
	}
	printf("week num of mine is %d \n",data_stream[WN].ori_eph);
	return data_stream[ZZZZZ].ori_eph + 1;
}
	
char *itoa(int val, char *buf, unsigned radix, int length)
{
    char   *p;             
    char   *firstdig;      
    char   temp;           
    unsigned   digval;     
    p = buf;
    if(val <0)
    {
        *p++ = '-';
        val = (unsigned long)(-(long)val);
    }
    firstdig = p; 
    do{
        digval = (unsigned)(val % radix);
        val /= radix;
       
        if  (digval > 9)
            *p++ = (char)(digval - 10 + 'a'); 
        else
            *p++ = (char)(digval + '0');      
    }while(val > 0);
	
    *p-- = '\0';         
    do{
        temp = *p;
        *p = *firstdig;
        *firstdig = temp;
        --p;
        ++firstdig;        
    }while(firstdig < p);  
    return buf;
}

char *itoa_with_len(int val, char *buf, unsigned radix, int length)
{
    char   *p;
    char   tmp_buf[48];
    char   *firstdig;      
    char   temp;           
    unsigned   digval;     
    int i;
    p = tmp_buf;
    if(val <0)
    {
        *p++ = '-';
        val = (unsigned long)(-(long)val);
    }
    firstdig = p; 
    do{
        digval = (unsigned)(val % radix);
        val /= radix;
        if  (digval > 9)
            *p++ = (char)(digval - 10 + 'a'); 
        else
            *p++ = (char)(digval + '0');      
    }while(val > 0);
	
    *p-- = '\0';         
    do{
        temp = *p;
        *p = *firstdig;
        *firstdig = temp;
        --p;
        ++firstdig;        
    }while(firstdig < p);

    printf("debug:: tmp buf is %s \n", tmp_buf);
	
	int len = strlen(tmp_buf);
	if(len <= length){
		if(tmp_buf[0] == '-'){
			for(i = 0; i <= (length - len); i++ )
				buf[i] = '1';
			for(i = 1; i < len - 1; i++ ){  //simple bu code generate,last bit keep
				if(tmp_buf[i] == '0')
					tmp_buf[i] = '1';
				else
					tmp_buf[i] = '0';
			}
			if(tmp_buf[len-1] == '0'){   // if have Cin, do some work
				for(i = len-2; i > 0; i--){
					if(tmp_buf[i] == '1'){
						tmp_buf[i] = '0';	
					}
					else{
						tmp_buf[i] = '1';
						break;
					}
				}
			}
			strcpy(buf + length - len +1, tmp_buf+1);//buf is the final result
		}
		else{
			for(i = 0; i < (length - len); i++ ) //positive , add 0 ahead
				buf[i] = '0';
			strcpy(buf + length - len, tmp_buf);
		}
	}
	else{
		printf("len of element is wrong,len is %d , buf is %s \n", len, tmp_buf);
	}

    printf("debug:: buf is %s \n", buf);
    return buf;
}

int convert2string(char eph[][33], _ori_data *data){
	char buf[48];
	int i;
	for(i=0;i<LAST_1;i++){
		itoa_with_len(data[i].ori_eph, buf, 2, data[i].width); //将真实数据转换为二进制字符串，保存于eph。
		strcpy(eph[i], buf);//strcpy have some doubt
	}
}

int update_dat_structure(char *nav_dat, char eph[][33], _ori_data *dat_info){ //刷新1500bit dat数据
	int i;
	for(i=0;i<LAST_1;i++){
		strncpy(nav_dat+dat_info[i].pos, eph[i], dat_info[i].width);//TODO::eph should added to _ori_data struct
	}
	write_tlm(nav_dat,TLM);
	printf("dat file is \n \n %s \n", dat);
}

int write_tlm(char *nav_dat,char *ok_tlm){
    int i;
    for(i=0;i< 1500;i+=300)
		strncpy(nav_dat + i,ok_tlm,strlen(ok_tlm));
	for(i=48;i< 1500;i+=300)
		nav_dat[i] = '1';
}

int check_parity(char *nav_dat, int zero_flag){
	char *last = nav_dat; //last 29
	char *new_line = nav_dat + 1; //last 30 , new_line+1 means 1
	int i,j;
	char sum = 0;
	char byte[32]; // temp buffer
		strncpy(byte,new_line+1,30);
		
		if(new_line[0] == '1'){     //if last one is 1, 1 -- 24 opsite 
			for(i=0;i<24;i++){
				if(byte[i] == '0')
					byte[i] = '1';
				else
					byte[i] = '0';
			}
		}
		/*for(j = 0; j < 15; j++){
			sum += ( last[D25[j]] - 48 );
		}
		printf("sum  25  is %d \n ", sum);
		sum = sum % 2;
		if(sum)
			byte[25-1]='1';
		else
			byte[25-1]='0';
		sum = 0;
		
		for(j = 0; j < 15; j++){
			sum += ( last[D26[j]] - 48 );
		}
		sum = sum % 2;
		if(sum)
			byte[26-1]='1';
		else
			byte[26-1]='0';
		sum = 0;
		
		for(j = 0; j < 15; j++){
			sum += ( last[D27[j]] - 48 );
		}
		sum = sum % 2;
		if(sum)
			byte[27-1]='1';
		else
			byte[27-1]='0';
		sum = 0;
		
		for(j = 0; j < 15; j++){
			sum += ( last[D28[j]] - 48 );
		}
		sum = sum % 2;
		if(sum)
			byte[28-1]='1';
		else
			byte[28-1]='0';
		sum = 0;*/

		for(j = 0; j < 16; j++){
			sum += ( last[D29[j]] - 48 );
		}
		sum = sum % 2;
		if(sum){
			byte[29-1]='1';
			if(zero_flag){ //D29 != 0,so ...
				last[24+1] = '1'; // modify d24 to make D29 to be 0
				byte[24-1] = ((1+last[1]) % 2) + 48; // caculate D24
				byte[29-1]='0';
			}
		}
		else
			byte[29-1]='0';
		sum = 0;
		
		for(j = 0; j < 14; j++){
			sum += ( last[D30[j]] - 48 );
		}
		sum = sum % 2;
		if(sum){
			byte[30-1]='1';
			if(zero_flag){ //D30 != 0,so ...
				last[23+1] = '1';
				byte[23-1]=((1+last[1]) % 2) + 48; // caculate D23
				byte[30-1]='0';
			}
		}
		else
			byte[30-1]='0';
		sum = 0;
		for(j = 0; j < 15; j++){
                        sum += ( last[D25[j]] - 48 );
                }
                printf("sum  25  is %d \n ", sum);
                sum = sum % 2;
                if(sum)
                        byte[25-1]='1';
                else
                        byte[25-1]='0';
                sum = 0;

                for(j = 0; j < 15; j++){
                        sum += ( last[D26[j]] - 48 );
                }
                sum = sum % 2;
                if(sum)
                        byte[26-1]='1';
                else
                        byte[26-1]='0';
                sum = 0;

                for(j = 0; j < 15; j++){
                        sum += ( last[D27[j]] - 48 );
                }
                sum = sum % 2;
                if(sum)
                        byte[27-1]='1';
                else
                        byte[27-1]='0';
                sum = 0;

                for(j = 0; j < 15; j++){
                        sum += ( last[D28[j]] - 48 );
                }
                sum = sum % 2;
                if(sum)
                        byte[28-1]='1';
                else
                        byte[28-1]='0';
                sum = 0;
		strncpy(new_line+1,byte,30);
}

int check_parity_all(char *nav_dat){
	char *tlm_end = nav_dat + 28;
	int i,j;
	for(j=0;j<5;j++){
		for(i=0;i<10;i++){
			if(i==0 || i==8)
				check_parity(tlm_end+30*i, 1);
			else
				check_parity(tlm_end+30*i, 0);
		}
		tlm_end += 300;
	}
}

int dup_update(char *nav_dat, int tow_cut){ //update cp 1500 bit, just update TOW & Parity
	char *z = nav_dat + 30;
	char *parity = nav_dat + 28;
	char buf[48];
	int i;
	for(i=0;i<5;i++){
		itoa_with_len(tow_cut + i, buf, 2, 17); //TOW CUT to 17 BITS
		strncpy(z, buf, 17);
		check_parity(parity, 1);
		z += 300;
		parity += 300; 
	}
}


int make_frame(nav_t *nav, int n, int min, char *filename){
	//int idx[32];
	//char dat[2048];
	if(min <= 0 || min > 120) //support 2h
		return -1;
	
	FILE *outfile;	
	int i,z_ret;
	_ori_data ori_data[LAST_1]; //nav data structure in message
	char string_eph[LAST_1][33]; //turn int to char,kept here
	
	outfile = fopen(filename, "w" );

	//find_eph_by_toe(toe,idx,nav);
	init_dat_files(DAT_LEN,dat);
	z_ret = init_original_data(nav, ori_data, n);
	convert2string(string_eph, ori_data);
	update_dat_structure(dat, string_eph, ori_data);
	check_parity_all(dat);
	for(i = 0;i < 2*min;i ++){
		fwrite(dat, sizeof(unsigned char), 1500, outfile);
		dup_update(dat,z_ret + 5*i);
	}

	fclose(outfile);
	printf("FINAL dat is \n %s \n", dat);
	printf("dat struct len is  %d \n", strlen(dat));
}

/*int main(int argc, char *argv[])
    
{
    FILE * outfile, *infile;
    int tlm = 0x00;    
    outfile = fopen(argv[2], "w" );   
    infile = fopen(argv[1], "rb");
    
    unsigned char buf[MAXLEN];
    
    if( argv[1] == NULL)
    {
    
        printf("%s, %s",argv[1],"not exit/n");
    
        exit(1);
    
    }   
    int rc;
    while( (rc = fread(buf,sizeof(unsigned char), 1,infile)) != 0 )
    {
    
        printf("sssssssssssssss is %s \n",buf);
		fwrite( buf, sizeof( unsigned char ), rc, outfile );
	}
	char *mybuf =	itoa(tlm,buf,2);
	printf("mybuf is %s\n", mybuf);	
	fwrite(buf, sizeof(unsigned int), 1, outfile);    

fclose(infile);
fclose(outfile);
    
return 0;
}*/
