from semantic_search.detection_fusion import auxiliary_candidate


def test_same_book_is_not_counted_twice_but_another_book_survives():
    primary=[('book',[0.,0.,100.,100.])]
    assert auxiliary_candidate('book',[2.,2.,98.,98.],primary,{'book'}) is None
    assert auxiliary_candidate('book',[120.,0.,200.,100.],primary,{'book'})==('book',73)


def test_explicit_scope_does_not_relabel_balloon_or_emit_spurious_apple():
    scope={'book','sports ball'};box=[0.,0.,100.,100.]
    assert auxiliary_candidate('exercise ball',box,[],scope)==('sports ball',32)
    assert auxiliary_candidate('balloon',box,[],scope) is None
    assert auxiliary_candidate('apple',box,[],scope) is None
    assert auxiliary_candidate('book',box,[],{'sports ball'}) is None
