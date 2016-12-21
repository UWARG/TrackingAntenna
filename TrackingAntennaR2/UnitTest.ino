#if UNIT_TEST
test(ScalarMult)
{
  int m;
  int n;
  float constant;
  float MatSource[10];
  float MatOutput[10];
  float expectedValue[10];
  int numTests = 3;

  for(int i=0; i < numTests; i++){

    if(i == 0){
      float test1MatSource[2][2] = {{1,1},{1,1}};
      float test1ExpectedValue[2][2] = {{5,5},{5,5}};
      constant = 5;
      m = 2;
      n = 2;

      memcpy(MatSource, test1MatSource, sizeof(test1MatSource));
      memcpy(expectedValue, test1ExpectedValue, sizeof(test1ExpectedValue));
    }

    if(i == 1){
      float test2MatSource[3][3] = {{1,1,1},{1,1,1},{1,1,1}};
      float test2ExpectedValue[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
      constant = 0;
      m = 3;
      n = 3;

      memcpy(MatSource, test2MatSource, sizeof(test2MatSource));
      memcpy(expectedValue, test2ExpectedValue, sizeof(test2ExpectedValue));
    }

    if(i == 2){
      float test3MatSource[3][2] = {{2,2},{2,2},{2,2}};
      float test3ExpectedValue[3][2] = {{4,4},{4,4},{4,4}};
      constant = 2;
      m = 3;
      n = 2;

      memcpy(MatSource, test3MatSource, sizeof(test3MatSource));
      memcpy(expectedValue, test3ExpectedValue, sizeof(test3ExpectedValue));    
    }
    
  }
  
  ScalarMult((float*)MatSource, constant, m, n, (float*)MatOutput);
  for(int i = 0; i < m; i++){
    for(int j = 0; j < n; j++){
      assertEqual(MatOutput[i * m + j], expectedValue[i * m + j]);
    }
  }
}


test(DotProduct) 
{
  int VecSize1;
  int VecSize2;
  float Vec1[10];
  float Vec2[10];
  float expectedValue;
  float testValue;
  int numTests = 3;

  for(int i = 0; i < numTests; i++){

    if(i == 0){
      float test1Vec1[3] = {1,2,3};
      float test1Vec2[3] = {1,2,3};
      VecSize1 = 3;
      VecSize2 = 3;
      expectedValue = 14;
    
      memcpy(Vec1, test1Vec1, sizeof(test1Vec1));
      memcpy(Vec2, test1Vec2, sizeof(test1Vec2));
    }

    if(i == 1){
      float test2Vec1[5] = {0,1,4,0,2};
      float test2Vec2[5] = {4,0,0,3,0};
      VecSize1 = 5;
      VecSize2 = 5;
      expectedValue = 0;

      memcpy(Vec1, test2Vec1, sizeof(test2Vec1));
      memcpy(Vec2, test2Vec2, sizeof(test2Vec2));
    }

    if(i == 2){
      float test3Vec1[2] = {1,2};
      float test3Vec2[3] = {3,2,1};
      VecSize1 = 2;
      VecSize2 = 3;
      expectedValue = 0;

      memcpy(Vec1, test3Vec1, sizeof(test3Vec1));
      memcpy(Vec2, test3Vec2, sizeof(test3Vec2));
    }

    testValue = DotProduct((float*)Vec1, (float*)Vec2, VecSize1, VecSize2);
    assertEqual(testValue, expectedValue);
  }
}
#endif
