# Contributing

This page is used as a starting point for contributing to the development of `gz-yarp-plugins`.

If you would like to contribute to the development of `gazebo-yarp-plugins`, please get in contact with the development team using [GitHub issues](https://github.com/robotology/gz-sim-yarp-plugins/issues).

If you need a new feature or to fix a bug, please [fill a GitHub issue](https://github.com/robotology/gz-sim-yarp-plugins/issues/new) describing the feature or the bugfix you would like to implement.

## Code style

We adopt a consistent code style.
If you want to contribute to this project, we kindly ask you to conform to the code style guidelines.

We do not enforce the style (at least at the current stage) but we will probably offer scripts to ease the process in the future.

More precisely, we try to use the **WebKit** coding style (see [WebKit page](http://www.webkit.org/coding/coding-style.html)).

## Patches and features contribution

The contribution follows the classical GitHub stages:

* open an issue (so we can discuss the problem and possible solutions);
* develop the changes;
* fork the project and submit the pull request.

## Repository structure and releases management

This part is mainly targeted to ''internal'' members.

Development of new features follows the [GitHub Flow](https://guides.github.com/introduction/flow/index.html) ( also know as [Feature Branch Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow) ).

In a nutshell, development of new features/bugfixes happens in separated branch. When you believe your contribution is stable
you can open a pull request against master, where your code will be review by other contributors prior to merging it.

[!NOTE]  
> **For maintainers:** Every time a new PR has to be merged in the `main` branch, the merge mode to use is [Squash and merge](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/incorporating-changes-from-a-pull-request/about-pull-request-merges#squash-and-merge-your-commits). This will help the blame of code changes, maintain a cleaner history on the `main` branch and facilitate code bisection for regression issues.
