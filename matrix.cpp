#include <iostream>
#include <cstring>

#define MAX 40

using namespace std;

class Matrix
{
public:
	int row=0,col=0;
	float data[MAX];
	Matrix(int r,int c){
		row=r;
		col=c;
	}
	void setMat(float t[MAX]){
		memcpy(data,t,row*col*sizeof(float));
	}
	float *element(int i,int j){
		return &data[i*col+j];
	}
	void transpose(Matrix *mtt){
		for (int i = 0; i < row; ++i)
		{
			for (int j = 0; j < col; ++j)
			{
				*mtt->element(j,i)=(*element(i,j));
			}
		}
	}
	void inverse(Matrix *mtt){
		for (int i = 0; i < row; ++i)
		{
			for (int j = 0; j < col; ++j)
			{
				*mtt->element(i,j)=1/(*element(i,j));
			}
		}
	}
	void add(Matrix *B,Matrix *C){
		for (int i = 0; i < row; ++i)
		{
			for (int j = 0; j < col; ++j)
			{
				*C->element(i,j)=(*B->element(i,j)) + (*element(i,j));
			}
		}
	}
	void dot(Matrix *B,Matrix *C){
		for (int i = 0; i < row; ++i)
		{
			for (int j = 0; j < B->col; ++j)
			{
				*C->element(i,j) = 0;
				for (int k = 0; k < col; k++){
					*C->element(i,j) += *element(i,k) * (*B->element(k,j));
				}
			}
		}
	}
};

int main()
{
	Matrix A(4,3);
	for (int i = 0; i < A.row; ++i)
	{
		for (int j = 0; j < A.col; ++j)
		{
			*A.element(i,j)=(float)i+1;
		}
	}
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			cout<<*A.element(i,j)<<" ";
		}
		cout<<endl;
	}
	Matrix B(3,4),C(4,4);
	A.transpose(&B);
	A.dot(&B,&C);
	cout<<endl;
	for (int i = 0; i < C.row; ++i)
	{
		for (int j = 0; j < C.col; ++j)
		{
			cout<<*C.element(i,j)<<" ";
		}
		cout<<endl;
	}
	return 0;
}