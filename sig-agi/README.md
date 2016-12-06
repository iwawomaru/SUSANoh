## Single Model Test
- shallow_a.h5 ... Accuracy:  0.7439
- shallow_b.h5 ... Accuracy:  0.7613
- thin.h5 ... Accuracy:  0.7131

## Average Eensemble Model Test
- shallow_a.h5 + shallow_b.h5 + thin.h5 (w/o softmax) ... Accuracy:  0.7947
- shallow_a.h5 + shallow_b.h5 + thin.h5 (with softmax) ... Accuracy:  0.7874
- shallow_a.h5 + shallow_b.h5 (with softmax) ... Accuracy:  0.7833

## Average Eensemble Model Test ith decay parameters
- shallow_b.h5 + shallow_a.h5 + thin.h5 (with softmax, [1, 0.99, 0.98]) ... Accuracy:  0.788
- thin.h5 + shallow_a.h5 + shallow_b.h5 (with softmax, [0.99, 0.995, 1]) ... Accuracy:  0.7881

## Hierarchical Eensemble Model Test
- shallow_b.h5 -> shallow_a.h5 -> thin.h5 (alpha=1) ... Accuracy:  0.7879
- thin.h5 -> shallow_a.h5 -> shalow_b.h5 (alpha=1.8) ... Accuracy:  0.7874

## Hierarchical Eensemble Model Test with decay parameters
- shallow_b.h5 -> shallow_a.h5 -> thin.h5 (alpha=1.4, beta=[1, 0.99, 0.98]) ... Accuracy:  0.7887
- thin.h5 -> shallow_a.h5 -> shalow_b.h5 (alpha=1.5, beta=[0.99, 0.995, 1]) ... Accuracy:  0.7881
