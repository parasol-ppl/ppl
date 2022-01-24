ifndef parallel
  parallel = 0
endif

ifndef Test
  Test = CfgTests
  #Test = ParallelCfgTests
endif

test:
	./tests.sh ${Test} ${parallel}

clean:
	rm -f Scratch/*
